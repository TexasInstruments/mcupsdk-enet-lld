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
 * \file  enetphy.c
 *
 * \brief This file contains the implementation of the Ethernet PHY driver.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <enet_cfg.h>
#include <include/core/enet_utils.h>
#include <priv/core/enet_trace_priv.h>
#include <include/phy/enetphy.h>
#include "enetphy_priv.h"
#include "generic_phy.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*! \brief Capability string buffer length. */
#define ENETPHY_CAPS_BUF_LEN                  (41U)

/*! \brief PHY mode string buffer length. */
#define ENETPHY_MODE_BUF_LEN                  (25U)

/*! \brief Default FINDING state timeout (in ticks). */
#define ENETPHY_TIMEOUT_FINDING               (20U)

/*! \brief Default RESET_WAIT state timeout (in ticks). */
#define ENETPHY_TIMEOUT_RESET_WAIT            (10U)

/*! \brief Default RESET_WAIT state residence time (in ticks). */
#define ENETPHY_RESIDENCE_RESET_WAIT          (0U)

/*! \brief Default LINK_WAIT state timeout (in ticks). */
#define ENETPHY_TIMEOUT_LINK_WAIT             (50U)

/*! \brief Default NWAY_START state timeout (in ticks). */
#define ENETPHY_TIMEOUT_NWAY_START            (50U)

/*! \brief Default NWAY_WAIT state timeout (in ticks). */
#define ENETPHY_TIMEOUT_NWAY_WAIT             (80U)

/*! \brief Default timeout if MDIX is enabled (in ticks). */
#define ENETPHY_TIMEOUT_MDIX                  (27U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

#if ((ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_WARN) && ENET_CFG_IS_OFF(TRACE_DISABLE_INFOSTRING))
static const char *EnetPhy_getCapsString(uint32_t linkCaps);
#endif

#if ((ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_DEBUG) && ENET_CFG_IS_OFF(TRACE_DISABLE_INFOSTRING))
static const char *EnetPhy_getModeString(EnetPhy_Speed speed,
                                         EnetPhy_Duplexity duplexity);
#endif

static EnetPhy_Handle EnetPhy_getHandle(void);

static void EnetPhy_releaseHandle(EnetPhy_Handle hPhy);

static void EnetPhy_setNextState(EnetPhy_Handle hPhy,
                                 EnetPhy_FsmState fsmState);

static void EnetPhy_phyTimeout(EnetPhy_Handle hPhy);

static void EnetPhy_initState(EnetPhy_Handle hPhy);

static void EnetPhy_defaultState(EnetPhy_Handle hPhy);

static void EnetPhy_findingState(EnetPhy_Handle hPhy);

static void EnetPhy_findingState(EnetPhy_Handle hPhy);

static void EnetPhy_foundState(EnetPhy_Handle hPhy);

static void EnetPhy_resetWaitState(EnetPhy_Handle hPhy);

static void EnetPhy_enableState(EnetPhy_Handle hPhy);

static void EnetPhy_setupNway(EnetPhy_Handle hPhy);

static void EnetPhy_setupManual(EnetPhy_Handle hPhy,
                                bool loopbackEn);

static void EnetPhy_loopbackState(EnetPhy_Handle hPhy);

static void EnetPhy_nwayStartState(EnetPhy_Handle hPhy);

static void EnetPhy_nwayWaitState(EnetPhy_Handle hPhy);

static void EnetPhy_linkWaitState(EnetPhy_Handle hPhy);

static void EnetPhy_linkedState(EnetPhy_Handle hPhy);

static bool EnetPhy_isNwayCapable(EnetPhy_Handle hPhy);

static uint32_t EnetPhy_getLocalCaps(EnetPhy_Handle hPhy);

static uint32_t EnetPhy_findCommonCaps(EnetPhy_Handle hPhy);

static uint32_t EnetPhy_findCommon1000Caps(EnetPhy_Handle hPhy);

static uint32_t EnetPhy_findCommonNwayCaps(EnetPhy_Handle hPhy);

static bool EnetPhy_isPhyLinked(EnetPhy_Handle hPhy);

static int32_t EnetPhy_bindDriver(EnetPhy_Handle hPhy);

static void EnetPhy_resetPhy(EnetPhy_Handle hPhy);

static uint32_t EnetPhy_getManualCaps(EnetPhy_Speed speed,
                                      EnetPhy_Duplexity duplexity);

static uint32_t EnetPhy_findBestCap(uint32_t caps);

static void EnetPhy_capToMode(uint32_t caps,
                              EnetPhy_Speed *speed,
                              EnetPhy_Duplexity *duplexity);

static void EnetPhy_showLinkPartnerCompat(EnetPhy_Handle hPhy,
                                          EnetPhy_Speed speed,
                                          EnetPhy_Duplexity duplexity);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


extern EnetPhy_DrvInfoTbl gEnetPhyDrvTbl;

/*! \brief Enet MAC port objects. */
static EnetPhy_Obj gEnetPhy_phyObjs[ENET_CFG_ENETPHY_PHY_MAX];

#if ((ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_DEBUG) && ENET_CFG_IS_OFF(TRACE_DISABLE_INFOSTRING))
/*! \brief Name of the FSM states. */
static const char *gEnetPhyStateNames[] =
{
    "INIT",
    "FINDING",
    "RESET_WAIT",
    "ENABLE",
    "FOUND",
    "NWAY_START",
    "NWAY_WAIT",
    "LINK_WAIT",
    "LINKED",
    "LOOPBACK",
};
#endif

#if ((ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_WARN) && ENET_CFG_IS_OFF(TRACE_DISABLE_INFOSTRING))
/*! \brief Ethernet PHY capabilities string buffer. */
static char gEnetPhyCapsBuf[ENETPHY_CAPS_BUF_LEN];
#endif

#if ((ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_DEBUG) && ENET_CFG_IS_OFF(TRACE_DISABLE_INFOSTRING))
/*! \brief Ethernet PHY mode string buffer. */
static char gEnetPhyModeBuf[ENETPHY_MODE_BUF_LEN];
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetPhy_initCfg(EnetPhy_Cfg *phyCfg)
{
    memset(phyCfg, 0, sizeof(*phyCfg));

    phyCfg->phyAddr = ENETPHY_INVALID_PHYADDR;
    phyCfg->nwayCaps = ENETPHY_LINK_CAP_ALL;
    phyCfg->mdixEn = true;
    phyCfg->isStrapped = false;
    phyCfg->loopbackEn = false;
    phyCfg->masterMode = false;
    phyCfg->extClkSource = false;
    phyCfg->skipExtendedCfg = false;
    phyCfg->timeoutCfg.findingStateTicks   = ENETPHY_TIMEOUT_FINDING;
    phyCfg->timeoutCfg.resetWaitStateTicks = ENETPHY_TIMEOUT_RESET_WAIT;
    phyCfg->timeoutCfg.resetWaitStateResidenceTicks = ENETPHY_RESIDENCE_RESET_WAIT;
    phyCfg->timeoutCfg.nwayStartStateTicks = ENETPHY_TIMEOUT_NWAY_START;
    phyCfg->timeoutCfg.nwayWaitStateTicks  = ENETPHY_TIMEOUT_NWAY_WAIT;
    phyCfg->timeoutCfg.linkWaitStateTicks  = ENETPHY_TIMEOUT_LINK_WAIT;
    phyCfg->timeoutCfg.mdixTicks           = ENETPHY_TIMEOUT_MDIX;
}

void EnetPhy_setExtendedCfg(EnetPhy_Cfg *phyCfg,
                            const void *extendedCfg,
                            uint32_t extendedCfgSize)
{
    Enet_devAssert(extendedCfgSize <= ENETPHY_EXTENDED_CFG_SIZE_MAX,
                   "Extended config size is too large: %u\r\n", extendedCfgSize);

    if ((extendedCfgSize > 0U) &&
        (extendedCfg != NULL))
    {
        memcpy(phyCfg->extendedCfg, extendedCfg, extendedCfgSize);
    }

    phyCfg->extendedCfgSize = extendedCfgSize;
}

EnetPhy_Handle EnetPhy_open(const EnetPhy_Cfg *phyCfg,
                            EnetPhy_Mii mii,
                            const EnetPhy_LinkCfg *linkCfg,
                            uint32_t macPortCaps,
                            EnetPhy_MdioHandle hMdio,
                            void *mdioArgs)
{
    EnetPhy_Handle hPhy = EnetPhy_getHandle();
    bool manualMode = false;
    bool alive;
    int32_t status = ENETPHY_SOK;

    if (hPhy == NULL)
    {
        ENETTRACE_ERR("PHY %u: Failed to allocate PHY object\r\n", phyCfg->phyAddr);
        status = ENETPHY_EALLOC;
    }

    if (status == ENET_SOK)
    {
        hPhy->hDrv    = NULL;
        hPhy->hMdio   = hMdio;
        hPhy->mdioArgs = mdioArgs;
        hPhy->macCaps = macPortCaps;
        hPhy->phyCfg  = *phyCfg;
        hPhy->mii     = mii;
        hPhy->linkCfg = *linkCfg;
        hPhy->addr    = phyCfg->phyAddr;
        hPhy->group   = phyCfg->phyGroup;

        /* State's linkCaps will be a mask for NWAY path and single bit for manual settings */
        if ((linkCfg->speed == ENETPHY_SPEED_AUTO) ||
            (linkCfg->duplexity == ENETPHY_DUPLEX_AUTO))
        {
            hPhy->reqLinkCaps = phyCfg->nwayCaps;
            if (hPhy->reqLinkCaps == 0U)
            {
                hPhy->reqLinkCaps = ENETPHY_LINK_CAP_ALL;
            }
        }
        else
        {
            hPhy->reqLinkCaps = EnetPhy_getManualCaps(linkCfg->speed, linkCfg->duplexity);
            manualMode = true;
        }

        hPhy->state.loopbackEn = phyCfg->loopbackEn;

        /* Take a shortcut to FOUND state if PHY is already alive, will save some
         * hundreds of msecs.  Take a shorter shortcut to LINK_WAIT state if PHY
         * is strap configured */
        alive = EnetPhy_isAlive(hPhy);
        if (alive)
        {
            EnetPhy_initState(hPhy);
            if (hPhy->phyCfg.isStrapped)
            {
                EnetPhy_setNextState(hPhy, ENETPHY_FSM_STATE_LINK_WAIT);
            }
            else
            {
                EnetPhy_setNextState(hPhy, ENETPHY_FSM_STATE_FOUND);
            }

            status = EnetPhy_bindDriver(hPhy);
        }
        else
        {
            EnetPhy_setNextState(hPhy, ENETPHY_FSM_STATE_INIT);
        }

        ENETTRACE_DBG_IF(status == ENETPHY_SOK, "PHY %u: open\r\n", hPhy->addr);

        /* Show compatible link partner configuration when local PHY is in manual mode */
        if (manualMode && !phyCfg->loopbackEn)
        {
            EnetPhy_showLinkPartnerCompat(hPhy, linkCfg->speed, linkCfg->duplexity);
        }

        /* Release allocated object in case of any errors */
        if (status != ENET_SOK)
        {
            EnetPhy_releaseHandle(hPhy);
        }
    }

    return (status == ENETPHY_SOK) ? hPhy : NULL;
}

void EnetPhy_close(EnetPhy_Handle hPhy)
{
    /* Power down and isolate the PHY if not strapped */
    if (!hPhy->phyCfg.isStrapped)
    {
        ENETTRACE_DBG("PHY %u: disable\r\n", hPhy->addr);
        EnetPhy_rmwReg(hPhy, PHY_BMCR,
                       BMCR_ISOLATE | BMCR_PWRDOWN,
                       BMCR_ISOLATE | BMCR_PWRDOWN);
    }

    EnetPhy_releaseHandle(hPhy);
}

EnetPhy_LinkStatus EnetPhy_tick(EnetPhy_Handle hPhy)
{
    EnetPhy_State *state = &hPhy->state;
    EnetPhy_FsmState prevFsmState;
    EnetPhy_FsmState currFsmState;
    EnetPhy_LinkStatus status;

    /* TODO: Check if pending state transitions needs to be applied,
     * i.e. from EnetPhy_setPhyMode() */

    /* State before tick */
    prevFsmState = state->fsmState;

    /*
     * Act on current state of the PHY.
     * The following fields of the FSM state get updated by the end of this
     * switch-case block:
     *  - fsmState: with the state for next tick: new or same state
     *  - fsmStateChanged: whether FSM will go to a new state or same state in
     *    next tick
     *  - timeout: Number of ticks remaning:
     *     o One tick less if still at the same state
     *     o The number of ticks for the new state as per open-time config
     */
    switch (state->fsmState)
    {
        case ENETPHY_FSM_STATE_INIT:
            ENETTRACE_VERBOSE("PHY %u: INIT\r\n", hPhy->addr);
            EnetPhy_initState(hPhy);
            break;

        case ENETPHY_FSM_STATE_FINDING:
            ENETTRACE_VERBOSE("PHY %u: FINDING (rem ticks %u)\r\n",
                              hPhy->addr, state->timeout);
            EnetPhy_findingState(hPhy);
            break;

        case ENETPHY_FSM_STATE_FOUND:
            ENETTRACE_VERBOSE("PHY %u: FOUND\r\n", hPhy->addr);
            EnetPhy_foundState(hPhy);
            break;

        case ENETPHY_FSM_STATE_RESET_WAIT:
            ENETTRACE_VERBOSE("PHY %u: RESET_WAIT\r\n", hPhy->addr);
            EnetPhy_resetWaitState(hPhy);
            break;

        case ENETPHY_FSM_STATE_ENABLE:
            ENETTRACE_VERBOSE("PHY %u: ENABLE\r\n", hPhy->addr);
            EnetPhy_enableState(hPhy);
            break;

        case ENETPHY_FSM_STATE_NWAY_START:
            ENETTRACE_VERBOSE("PHY %u: NWAY_START (rem ticks %u)\r\n", hPhy->addr, state->timeout);
            EnetPhy_nwayStartState(hPhy);
            break;

        case ENETPHY_FSM_STATE_NWAY_WAIT:
            ENETTRACE_VERBOSE("PHY %u: NWAY_WAIT (rem ticks %u)\r\n", hPhy->addr, state->timeout);
            EnetPhy_nwayWaitState(hPhy);
            break;

        case ENETPHY_FSM_STATE_LINK_WAIT:
            ENETTRACE_VERBOSE("PHY %u: LINK_WAIT (rem ticks %u)\r\n", hPhy->addr, state->timeout);
            EnetPhy_linkWaitState(hPhy);
            break;

        case ENETPHY_FSM_STATE_LINKED:
            ENETTRACE_VERBOSE("PHY %u: LINKED\r\n", hPhy->addr);
            EnetPhy_linkedState(hPhy);
            break;

        case ENETPHY_FSM_STATE_LOOPBACK:
            ENETTRACE_VERBOSE("PHY %u: LOOPBACK\r\n", hPhy->addr);
            EnetPhy_loopbackState(hPhy);
            break;

        default:
            ENETTRACE_VERBOSE("PHY %u: DEFAULT\r\n", hPhy->addr);
            EnetPhy_defaultState(hPhy);
            break;
    }

    /* State after tick */
    currFsmState = state->fsmState;

    /*
     * New link status:
     *  - Just got link (ENETPHY_GOT_LINK)
     *  - Already had link (ENETPHY_LINK_UP)
     *  - Just lost link (ENETPHY_LOST_LINK)
     *  - Link is still down (ENETPHY_LINK_DOWN)
     */
    switch (currFsmState)
    {
        case ENETPHY_FSM_STATE_LINKED:
        case ENETPHY_FSM_STATE_LOOPBACK:
            if (prevFsmState != currFsmState)
            {
                status = ENETPHY_GOT_LINK;
            }
            else
            {
                status = ENETPHY_LINK_UP;
            }
            break;

        case ENETPHY_FSM_STATE_INIT:
        case ENETPHY_FSM_STATE_FINDING:
        case ENETPHY_FSM_STATE_FOUND:
        case ENETPHY_FSM_STATE_RESET_WAIT:
        case ENETPHY_FSM_STATE_ENABLE:
        case ENETPHY_FSM_STATE_NWAY_START:
        case ENETPHY_FSM_STATE_NWAY_WAIT:
        case ENETPHY_FSM_STATE_LINK_WAIT:
        default:
            if ((prevFsmState == ENETPHY_FSM_STATE_LINKED) ||
                (prevFsmState == ENETPHY_FSM_STATE_LOOPBACK))
            {
                status = ENETPHY_LOST_LINK;
            }
            else
            {
                status = ENETPHY_LINK_DOWN;
            }
            break;
    }

    return status;
}

bool EnetPhy_isAlive(EnetPhy_Handle hPhy)
{
    EnetPhy_MdioHandle hMdio = hPhy->hMdio;
    uint32_t phyGroup = hPhy->group;
    uint32_t phyAddr = hPhy->addr;
    bool isAlive = false;
    uint16_t val = 0U;
    int32_t status;

    /* Get PHY alive status */
    if (hMdio->isAlive != NULL)
    {
        /* Get alive status from MDIO driver (i.e. hardware assisted) */
        status = hMdio->isAlive(phyAddr, &isAlive, hPhy->mdioArgs);
        ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                         "PHY %u: Failed to get alive status: %d\r\n", phyAddr, status);
    }
    else
    {
        /* Alternatively, read BMSR - PHY is alive if transaction is successful */
        status = hMdio->readC22(phyGroup, phyAddr, PHY_BMSR, &val, hPhy->mdioArgs);
        ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                         "PHY %u: Failed to read reg %u: %d\r\n", phyAddr, PHY_BMSR, status);
        if (status == ENETPHY_SOK)
        {
            isAlive = true;
        }
    }

    return isAlive;
}

int32_t EnetPhy_getId(EnetPhy_Handle hPhy,
                      EnetPhy_Version *version)
{
    uint32_t phyAddr = hPhy->addr;
    uint16_t id1, id2;
    int32_t status = ENETPHY_SOK;
    bool alive;

    ENETTRACE_VAR(phyAddr);
    alive = EnetPhy_isAlive(hPhy);
    if (alive == false)
    {
        status = ENETPHY_EFAIL;
    }

    if (status == ENETPHY_SOK)
    {
        status = EnetPhy_readReg(hPhy, PHY_PHYIDR1, &id1);
        ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                         "PHY %u: Failed to read ID1 register: %d\r\n", phyAddr, status);
    }

    if (status == ENETPHY_SOK)
    {
        status = EnetPhy_readReg(hPhy, PHY_PHYIDR2, &id2);
        ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                         "PHY %u: Failed to read ID2 register: %d\r\n", phyAddr, status);
    }

    if (status == ENETPHY_SOK)
    {
        version->oui      = ((uint32_t)id1 << PHYIDR1_OUI_OFFSET);
        version->oui     |= (id2 & PHYIDR2_OUI_MASK)  >> PHYIDR2_OUI_OFFSET;
        version->model    = (id2 & PHYIDR2_VMDL_MASK) >> PHYIDR2_VMDL_OFFSET;
        version->revision = (id2 & PHYIDR2_VREV_MASK) >> PHYIDR2_VREV_OFFSET;
    }

    return status;
}

bool EnetPhy_isLinked(EnetPhy_Handle hPhy)
{
    EnetPhy_State *state = &hPhy->state;
    EnetPhy_FsmState fsmState = state->fsmState;
    EnetPhy_FsmState linkupFsmState;
    bool isLinked;

    /* Link up is reached in LINKED or LOOPBACK states */
    linkupFsmState = state->loopbackEn ?
                    ENETPHY_FSM_STATE_LOOPBACK : ENETPHY_FSM_STATE_LINKED;
    isLinked = (fsmState == linkupFsmState);
    return isLinked;
}

int32_t EnetPhy_getLinkCfg(EnetPhy_Handle hPhy,
                           EnetPhy_LinkCfg *linkCfg)
{
    EnetPhy_State *state = &hPhy->state;
    bool isLinked;
    int32_t status = ENETPHY_SOK;

    isLinked = EnetPhy_isLinked(hPhy);
    if (isLinked)
    {
        linkCfg->speed = state->speed;
        linkCfg->duplexity = state->duplexity;
    }
    else
    {
        ENETTRACE_WARN("PHY %u: PHY is not linked, can't get link config\r\n", hPhy->addr);
        status = ENETPHY_EPERM;
    }
    return status;
}

int32_t EnetPhy_readReg(EnetPhy_Handle hPhy,
                        uint32_t reg,
                        uint16_t *val)
{
    EnetPhy_MdioHandle hMdio = hPhy->hMdio;
    uint32_t phyGroup = hPhy->group;
    uint32_t phyAddr = hPhy->addr;
    int32_t status;

    status = hMdio->readC22(phyGroup, phyAddr, reg, val, hPhy->mdioArgs);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "PHY %u: Failed to read reg %u: %d\r\n", phyAddr, reg, status);
    ENETTRACE_VERBOSE_IF(status == ENETPHY_SOK,
                         "PHY %u: reg %u val 0x%04x %s\r\n", phyAddr, reg, *val);

    return status;
}

int32_t EnetPhy_writeReg(EnetPhy_Handle hPhy,
                         uint32_t reg,
                         uint16_t val)
{
    EnetPhy_MdioHandle hMdio = hPhy->hMdio;
    uint32_t phyGroup = hPhy->group;
    uint32_t phyAddr = hPhy->addr;
    int32_t status;

    status = hMdio->writeC22(phyGroup, phyAddr, reg, val, hPhy->mdioArgs);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "PHY %u: Failed to write reg %u: %d\r\n", phyAddr, reg, status);
    ENETTRACE_VERBOSE_IF(status == ENETPHY_SOK,
                         "PHY %u: reg %u val 0x%04x\r\n", phyAddr, reg, val);

    return status;
}

int32_t EnetPhy_rmwReg(EnetPhy_Handle hPhy,
                       uint32_t reg,
                       uint16_t mask,
                       uint16_t val)
{
    EnetPhy_MdioHandle hMdio = hPhy->hMdio;
    uint32_t phyGroup = hPhy->group;
    uint32_t phyAddr = hPhy->addr;
    int32_t status;
    uint16_t data = 0U;

    status = hMdio->readC22(phyGroup, phyAddr, reg, &data, hPhy->mdioArgs);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "PHY %u: Failed to read reg %u: %d\r\n", phyAddr, reg, status);
    ENETTRACE_VERBOSE_IF(status == ENETPHY_SOK,
                         "PHY %u: read reg %u val 0x%04x\r\n", phyAddr, reg, data);

    if (status == ENETPHY_SOK)
    {
        data = (data & ~mask) | (val & mask);

        status = hMdio->writeC22(phyGroup, phyAddr, reg, data, hPhy->mdioArgs);
        ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                         "PHY %u: Failed to write reg %u: %d\r\n", phyAddr, reg, status);
        ENETTRACE_VERBOSE_IF(status == ENETPHY_SOK,
                             "PHY %u: write reg %u val 0x%04x\r\n", phyAddr, reg, data);
    }

    return status;
}

int32_t EnetPhy_readExtReg(EnetPhy_Handle hPhy,
                           uint32_t reg,
                           uint16_t *val)
{
    int32_t status = ENETPHY_ENOTSUPPORTED;

    if ((hPhy->hDrv != NULL) &&
        (hPhy->hDrv->readExtReg != NULL))
    {
        status = hPhy->hDrv->readExtReg(hPhy, reg, val);
    }

    return status;
}

int32_t EnetPhy_writeExtReg(EnetPhy_Handle hPhy,
                            uint32_t reg,
                            uint16_t val)
{
    int32_t status = ENETPHY_ENOTSUPPORTED;

    if ((hPhy->hDrv != NULL) &&
        (hPhy->hDrv->writeExtReg != NULL))
    {
        status = hPhy->hDrv->writeExtReg(hPhy, reg, val);
    }

    return status;
}

int32_t EnetPhy_rmwExtReg(EnetPhy_Handle hPhy,
                          uint32_t reg,
                          uint16_t mask,
                          uint16_t val)
{
    int32_t status = ENETPHY_ENOTSUPPORTED;
    uint16_t data = 0U;

    if ((hPhy->hDrv != NULL) &&
        (hPhy->hDrv->readExtReg != NULL) &&
        (hPhy->hDrv->writeExtReg != NULL))
    {
        status = hPhy->hDrv->readExtReg(hPhy, reg, &data);

        if (status == ENETPHY_SOK)
        {
            data = (data & ~mask) | (val & mask);

            status = hPhy->hDrv->writeExtReg(hPhy, reg, data);
        }
    }

    return status;
}

int32_t EnetPhy_readC45Reg(EnetPhy_Handle hPhy,
                           uint8_t mmd,
                           uint32_t reg,
                           uint16_t *val)
{
    EnetPhy_MdioHandle hMdio = hPhy->hMdio;
    uint32_t phyGroup = hPhy->group;
    uint32_t phyAddr = hPhy->addr;
    int32_t status;

    status = hMdio->readC45(phyGroup, phyAddr, mmd, reg, val, hPhy->mdioArgs);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "PHY %u: Failed to read MMD %u reg %u: %d\r\n", phyAddr, mmd, reg, status);
    ENETTRACE_VERBOSE_IF(status == ENETPHY_SOK,
                         "PHY %u: MMD %u reg %u val 0x%04x %s\r\n", phyAddr, mmd, reg, *val);

    return status;
}

int32_t EnetPhy_writeC45Reg(EnetPhy_Handle hPhy,
                            uint8_t mmd,
                            uint32_t reg,
                            uint16_t val)
{
    EnetPhy_MdioHandle hMdio = hPhy->hMdio;
    uint32_t phyGroup = hPhy->group;
    uint32_t phyAddr = hPhy->addr;
    int32_t status;

    status = hMdio->writeC45(phyGroup, phyAddr, mmd, reg, val, hPhy->mdioArgs);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "PHY %u: Failed to write MMD %u reg %u: %d\r\n", phyAddr, mmd, reg, status);
    ENETTRACE_VERBOSE_IF(status == ENETPHY_SOK,
                         "PHY %u: MMD %u reg %u val 0x%04x\r\n", phyAddr, mmd, reg, val);

    return status;
}

int32_t EnetPhy_rmwC45Reg(EnetPhy_Handle hPhy,
                          uint8_t mmd,
                          uint32_t reg,
                          uint16_t mask,
                          uint16_t val)
{
    EnetPhy_MdioHandle hMdio = hPhy->hMdio;
    uint32_t phyGroup = hPhy->group;
    uint32_t phyAddr = hPhy->addr;
    int32_t status;
    uint16_t data = 0U;

    status = hMdio->readC45(phyGroup, phyAddr, mmd, reg, &data, hPhy->mdioArgs);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "PHY %u: Failed to read MMD %u reg %u: %d\r\n", phyAddr, mmd, reg, status);
    ENETTRACE_VERBOSE_IF(status == ENETPHY_SOK,
                         "PHY %u: read MMD %u reg %u val 0x%04x\r\n", phyAddr, mmd, reg, data);

    if (status == ENETPHY_SOK)
    {
        data = (data & ~mask) | (val & mask);

        status = hMdio->writeC45(phyGroup, phyAddr, mmd, reg, data, hPhy->mdioArgs);
        ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                         "PHY %u: Failed to write MMD %u reg %u: %d\r\n", phyAddr, mmd, reg, status);
        ENETTRACE_VERBOSE_IF(status == ENETPHY_SOK,
                             "PHY %u: write MMD %u reg %u val 0x%04x\r\n", phyAddr, mmd, reg, data);
    }

    return status;
}

void EnetPhy_printRegs(EnetPhy_Handle hPhy)
{
    if ((hPhy->hDrv != NULL) &&
        (hPhy->hDrv->printRegs != NULL))
    {
        hPhy->hDrv->printRegs(hPhy);
    }
}

#if ((ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_WARN) && ENET_CFG_IS_OFF(TRACE_DISABLE_INFOSTRING))
static const char *EnetPhy_getCapsString(uint32_t linkCaps)
{
    snprintf(gEnetPhyCapsBuf, ENETPHY_CAPS_BUF_LEN, "%s%s%s%s%s%s%s",
             ((linkCaps & ENETPHY_LINK_CAP_FD1000) != 0U) ? "FD1000 " : "",
             ((linkCaps & ENETPHY_LINK_CAP_HD1000) != 0U) ? "HD1000 " : "",
             ((linkCaps & ENETPHY_LINK_CAP_FD100) != 0U) ? "FD100 " : "",
             ((linkCaps & ENETPHY_LINK_CAP_HD100) != 0U) ? "HD100 " : "",
             ((linkCaps & ENETPHY_LINK_CAP_FD10) != 0U) ? "FD10 " : "",
             ((linkCaps & ENETPHY_LINK_CAP_HD10) != 0U) ? "HD10 " : "",
             ((linkCaps & ENETPHY_LINK_CAP_ALL) == 0U) ? "None" : "");

    return gEnetPhyCapsBuf;
}
#endif

#if ((ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_DEBUG) && ENET_CFG_IS_OFF(TRACE_DISABLE_INFOSTRING))
static const char *EnetPhy_getModeString(EnetPhy_Speed speed,
                                         EnetPhy_Duplexity duplexity)
{
    snprintf(gEnetPhyModeBuf, ENETPHY_MODE_BUF_LEN, "%sbps %s-duplex",
             (speed == ENETPHY_SPEED_1GBIT) ? "1 G" :
             (speed == ENETPHY_SPEED_100MBIT) ? "100 M" : "10 M",
             (duplexity == ENETPHY_DUPLEX_FULL) ? "full" : "half");

    return gEnetPhyModeBuf;
}
#endif

static EnetPhy_Handle EnetPhy_getHandle(void)
{
    EnetPhy_Handle hPhy = NULL;
    uint32_t i;

    for (i = 0U; i < ENETPHY_ARRAYSIZE(gEnetPhy_phyObjs); i++)
    {
        if (gEnetPhy_phyObjs[i].magic != ENETPHY_MAGIC)
        {
            hPhy = &gEnetPhy_phyObjs[i];
            memset(hPhy, 0, sizeof(*hPhy));
            hPhy->magic = ENETPHY_MAGIC;
            break;
        }
    }

    return hPhy;
}

static void EnetPhy_releaseHandle(EnetPhy_Handle hPhy)
{
    hPhy->magic = ENETPHY_NO_MAGIC;
}

static void EnetPhy_setNextState(EnetPhy_Handle hPhy,
                                 EnetPhy_FsmState fsmState)
{
    EnetPhy_FsmTimeoutCfg *timeoutCfg = &hPhy->phyCfg.timeoutCfg;
    EnetPhy_FsmState prevFsmState = hPhy->state.fsmState;
    uint32_t timeout;
    uint32_t residenceTime = 0U;

    switch (fsmState)
    {
        case ENETPHY_FSM_STATE_INIT:
            timeout = 0U;
            break;

        case ENETPHY_FSM_STATE_FINDING:
            timeout = timeoutCfg->findingStateTicks;
            break;

        case ENETPHY_FSM_STATE_FOUND:
            timeout = 0U;
            break;

        case ENETPHY_FSM_STATE_RESET_WAIT:
            timeout = timeoutCfg->resetWaitStateTicks;
            residenceTime = timeoutCfg->resetWaitStateResidenceTicks;
            break;

        case ENETPHY_FSM_STATE_ENABLE:
            timeout = 0U;
            break;

        case ENETPHY_FSM_STATE_NWAY_START:
            timeout = timeoutCfg->nwayStartStateTicks;
            break;

        case ENETPHY_FSM_STATE_NWAY_WAIT:
            timeout = timeoutCfg->nwayWaitStateTicks;
            break;

        case ENETPHY_FSM_STATE_LINK_WAIT:
            timeout = timeoutCfg->linkWaitStateTicks;
            break;

        case ENETPHY_FSM_STATE_LINKED:
            timeout = 0U;
            break;

        case ENETPHY_FSM_STATE_LOOPBACK:
            timeout = 0U;
            break;

        default:
            timeout = 0U;
            break;
    }

    ENETTRACE_DBG("PHY %u: %s -> %s (%u ticks)\r\n",
                  hPhy->addr,
                  gEnetPhyStateNames[hPhy->state.fsmState],
                  gEnetPhyStateNames[fsmState],
                  timeout);

    hPhy->state.fsmState = fsmState;
    hPhy->state.timeout = timeout;
    hPhy->state.residenceTime = residenceTime;
    hPhy->state.fsmStateChanged = (fsmState != prevFsmState);
}

static void EnetPhy_phyTimeout(EnetPhy_Handle hPhy)
{
    ENETTRACE_DBG("PHY %u: timeout has occurred\r\n", hPhy->addr);

    /* Reset state machine to FOUND for non-strapped PHY, or to LINK_WAIT for
     * state for strapped PHY as most FSM states are bypassed */
    if (hPhy->phyCfg.isStrapped)
    {
        EnetPhy_setNextState(hPhy, ENETPHY_FSM_STATE_LINK_WAIT);
    }
    else
    {
        EnetPhy_setNextState(hPhy, ENETPHY_FSM_STATE_FOUND);
    }
}

static void EnetPhy_initState(EnetPhy_Handle hPhy)
{
    EnetPhy_State *state = &hPhy->state;

    state->speed = ENETPHY_SPEED_10MBIT;
    state->duplexity = ENETPHY_DUPLEX_HALF;
    state->phyLinkCaps = 0U;

    if (hPhy->phyCfg.isStrapped)
    {
        state->needsManualCfg = false;
        state->needsNwayCfg   = false;
    }
    else
    {
        state->needsManualCfg = true;
        state->needsNwayCfg   = true;
    }

    EnetPhy_setNextState(hPhy, ENETPHY_FSM_STATE_FINDING);
}

static void EnetPhy_defaultState(EnetPhy_Handle hPhy)
{
    hPhy->state.fsmStateChanged = true;
}

static void EnetPhy_findingState(EnetPhy_Handle hPhy)
{
    EnetPhy_State *state = &hPhy->state;
    uint32_t phyAddr = hPhy->addr;
    int32_t status;
    bool alive;

    ENETTRACE_VAR(phyAddr);
    alive = EnetPhy_isAlive(hPhy);
    if (alive)
    {
        /* PHY found */
        ENETTRACE_DBG("PHY %u: PHY has been found\r\n", phyAddr);

        status = EnetPhy_bindDriver(hPhy);
        if (status == ENETPHY_SOK)
        {
            if (hPhy->phyCfg.isStrapped)
            {
                EnetPhy_setNextState(hPhy, ENETPHY_FSM_STATE_LINK_WAIT);
            }
            else
            {
                EnetPhy_setNextState(hPhy, ENETPHY_FSM_STATE_FOUND);
            }
        }
        else
        {
            /* TODO: Go to a 'disable' state */
            ENETTRACE_ERR("PHY %u: couldn't be bound to any driver\r\n", phyAddr);
        }
    }
    else
    {
        if (state->timeout != 0U)
        {
            state->timeout--;
        }
        else
        {
            ENETTRACE_DBG("PHY %u: timed out\r\n", phyAddr);
            state->timeout = ENETPHY_TIMEOUT_FINDING;
        }
    }
}

static void EnetPhy_foundState(EnetPhy_Handle hPhy)
{
    Enet_devAssert(!hPhy->phyCfg.isStrapped,
                   "PHY %u: unexpected state for strapped PHY\r\n", hPhy->addr);

    EnetPhy_resetPhy(hPhy);

    EnetPhy_setNextState(hPhy, ENETPHY_FSM_STATE_RESET_WAIT);
}

static void EnetPhy_resetWaitState(EnetPhy_Handle hPhy)
{
    EnetPhy_State *state = &hPhy->state;
    bool complete;

    Enet_devAssert(hPhy->hDrv->isResetComplete != NULL,
                   "PHY %u: isResetComplete callback not implemented\r\n", hPhy->addr);
    Enet_devAssert(!hPhy->phyCfg.isStrapped,
                   "PHY %u: unexpected state for strapped PHY\r\n", hPhy->addr);

    /* Wait for PHY reset to complete */
    complete = hPhy->hDrv->isResetComplete(hPhy);
    if (complete)
    {
        if (state->residenceTime != 0U)
        {
            state->residenceTime--;
        }
        else
        {
            EnetPhy_setNextState(hPhy, ENETPHY_FSM_STATE_ENABLE);
        }
    }
    else
    {
        if (state->timeout != 0U)
        {
            state->timeout--;
        }
        else
        {
            EnetPhy_phyTimeout(hPhy);
        }
    }
}

static void EnetPhy_enableState(EnetPhy_Handle hPhy)
{
    EnetPhy_State *state = &hPhy->state;
    uint32_t socLinkCaps;
    uint32_t commonLinkCaps;


    Enet_devAssert(!hPhy->phyCfg.isStrapped,
                   "PHY %u: unexpected state for strapped PHY\r\n", hPhy->addr);

    /* Enable PHY (resetting it if applicable) can be done only when
     * entering the ENABLE state for the first time */
    if (state->needsNwayCfg ||
        state->needsManualCfg)
    {
        /* Set PHY in normal mode */
        ENETTRACE_DBG("PHY %u: enable\r\n", hPhy->addr);
        EnetPhy_rmwReg(hPhy, PHY_BMCR, BMCR_ISOLATE | BMCR_PWRDOWN, 0U);

        /* PHY-specific 'extended' configuration */
        if ((hPhy->hDrv->config != NULL) &&
            !hPhy->phyCfg.skipExtendedCfg)
        {
            hPhy->hDrv->config(hPhy, &hPhy->phyCfg, hPhy->mii);
        }
    }

    /* Find common capabilities among app's, PHY's and SoC's */
    ENETTRACE_DBG("PHY %u: req caps: %s\r\n",
                  hPhy->addr, EnetPhy_getCapsString(hPhy->reqLinkCaps));

    state->phyLinkCaps = EnetPhy_getLocalCaps(hPhy);
    ENETTRACE_DBG("PHY %u: PHY caps: %s\r\n",
                  hPhy->addr, EnetPhy_getCapsString(state->phyLinkCaps));

    socLinkCaps = hPhy->macCaps;
    ENETTRACE_DBG("PHY %u: MAC caps: %s\r\n",
                  hPhy->addr, EnetPhy_getCapsString(socLinkCaps));

    commonLinkCaps = hPhy->reqLinkCaps &
                     state->phyLinkCaps &
                     socLinkCaps;
    ENETTRACE_DBG("PHY %u: refined caps: %s\r\n",
                  hPhy->addr, EnetPhy_getCapsString(commonLinkCaps));

    if (commonLinkCaps != 0U)
    {
        state->linkCaps = commonLinkCaps;

        /* Enable NWAY if requested by app (via auto-speed or auto-duplex) and
         * supported by PHY */
        state->isNwayCapable = EnetPhy_isNwayCapable(hPhy);
        ENETTRACE_DBG("PHY %u: PHY is %sNWAY-capable\r\n",
                      hPhy->addr, state->isNwayCapable ? "" : "not ");

        state->enableNway = (hPhy->linkCfg.speed == ENETPHY_SPEED_AUTO) ||
                            (hPhy->linkCfg.duplexity == ENETPHY_DUPLEX_AUTO);
        if (state->enableNway && !state->isNwayCapable)
        {
            /* If PHY is not capable of auto negotiation, fallback to manual mode
             * with the highest refined capability */
            ENETTRACE_WARN("PHY %u: falling back to manual mode\r\n", hPhy->addr);
            state->enableNway = false;

            commonLinkCaps = EnetPhy_findBestCap(commonLinkCaps);
            ENETTRACE_WARN("PHY %u: new link caps: %s\r\n",
                           hPhy->addr, EnetPhy_getCapsString(commonLinkCaps));
            state->linkCaps = commonLinkCaps;
        }
    }

    if (commonLinkCaps == 0U)
    {
        /* TODO: Should it go to a new "disabled" state where state machine doesn't run */
        ENETTRACE_ERR("PHY %u: no supported caps found\r\n", hPhy->addr);
    }
    else if (state->loopbackEn)
    {
        ENETTRACE_DBG("PHY %u: setup loopback\r\n", hPhy->addr);
        EnetPhy_setupManual(hPhy, true);
        EnetPhy_setNextState(hPhy, ENETPHY_FSM_STATE_LOOPBACK);
    }
    else if (state->isNwayCapable && state->enableNway)
    {
        ENETTRACE_DBG("PHY %u: setup NWAY\r\n", hPhy->addr);
        EnetPhy_setupNway(hPhy);
        EnetPhy_setNextState(hPhy, ENETPHY_FSM_STATE_NWAY_START);
    }
    else
    {
        ENETTRACE_DBG("PHY %u: manual setup\r\n", hPhy->addr);
        EnetPhy_setupManual(hPhy, false);
        EnetPhy_setNextState(hPhy, ENETPHY_FSM_STATE_LINK_WAIT);
    }
}

static void EnetPhy_setupNway(EnetPhy_Handle hPhy)
{
    EnetPhy_State *state = &hPhy->state;
    uint16_t nwayAdvertise;
    uint16_t nway1000Advertise = 0U;

    ENETTRACE_DBG("PHY %u: NWAY advertising: %s\r\n",
                  hPhy->addr, EnetPhy_getCapsString(state->linkCaps));

    ENETTRACE_DBG("PHY %u: config is %sneeded\r\n",
                  hPhy->addr, state->needsNwayCfg ? "" : "not ");

    if (state->needsNwayCfg)
    {
        nwayAdvertise = ANAR_802P3;
        if ((state->linkCaps & ENETPHY_LINK_CAP_FD100) != 0U)
        {
            nwayAdvertise |= ANAR_100FD;
        }

        if ((state->linkCaps & ENETPHY_LINK_CAP_HD100) != 0U)
        {
            nwayAdvertise |= ANAR_100HD;
        }

        if ((state->linkCaps & ENETPHY_LINK_CAP_FD10) != 0U)
        {
            nwayAdvertise |= ANAR_10FD;
        }

        if ((state->linkCaps & ENETPHY_LINK_CAP_HD10) != 0U)
        {
            nwayAdvertise |= ANAR_10HD;
        }

        if ((state->linkCaps & ENETPHY_LINK_CAP_FD1000) != 0U)
        {
            nway1000Advertise |= GIGCR_1000FD;
        }

        if ((state->linkCaps & ENETPHY_LINK_CAP_HD1000) != 0U)
        {
            nway1000Advertise |= GIGCR_1000HD;
        }

        EnetPhy_rmwReg(hPhy, PHY_ANAR, ANAR_100 | ANAR_10, nwayAdvertise);

        if ((state->phyLinkCaps & ENETPHY_LINK_CAP_1000) != 0U)
        {
            EnetPhy_rmwReg(hPhy, PHY_GIGCR, GIGCR_1000, nway1000Advertise);
        }

        state->needsNwayCfg = false;
    }

    /* Restart auto-negotiation */
    ENETTRACE_DBG("PHY %u: restart autonegotiation\r\n", hPhy->addr);
    EnetPhy_rmwReg(hPhy, PHY_BMCR, BMCR_ANEN, BMCR_ANEN);

    /* TODO: is MII_ENETPHY_FD needed for auto-negotiation? */
    EnetPhy_rmwReg(hPhy, PHY_BMCR,
                   BMCR_ANRESTART | BMCR_FD,
                   BMCR_ANRESTART | BMCR_FD);
}

static void EnetPhy_setupManual(EnetPhy_Handle hPhy,
                                bool loopbackEn)
{
    EnetPhy_State *state = &hPhy->state;
    uint16_t val;

    /* Manual config is needed if entered the FOUND state for the first time
     * or if an explicit mode change was requested via EnetPhy_setPhyMode() */
    if (state->needsManualCfg)
    {
        /* Disable auto-negotiation if supported by PHY */
        if (state->isNwayCapable)
        {
            ENETTRACE_DBG("PHY %u: disable NWAY\r\n", hPhy->addr);
            EnetPhy_rmwReg(hPhy, PHY_BMCR, BMCR_ANEN, 0U);
        }

        switch (state->linkCaps)
        {
            case ENETPHY_LINK_CAP_FD1000:
                val = BMCR_SPEED1000 | BMCR_FD;
                break;

            case ENETPHY_LINK_CAP_HD1000:
                val = BMCR_SPEED1000;
                break;

            case ENETPHY_LINK_CAP_FD100:
                val = BMCR_SPEED100 | BMCR_FD;
                break;

            case ENETPHY_LINK_CAP_HD100:
                val = BMCR_SPEED100;
                break;

            case ENETPHY_LINK_CAP_FD10:
                val = BMCR_FD;
                break;

            case ENETPHY_LINK_CAP_HD10:
            default:
                val = 0U;
                break;
        }

        if (loopbackEn)
        {
            val |= BMCR_LOOPBACK;
        }

        EnetPhy_rmwReg(hPhy, PHY_BMCR,
                       BMCR_LOOPBACK | BMCR_SPEED1000 | BMCR_SPEED100 | BMCR_FD,
                       val);

        /* Clear advertised capabilities values if supported */
        if (state->isNwayCapable)
        {
            EnetPhy_rmwReg(hPhy, PHY_ANAR, ANAR_100 | ANAR_10, 0U);

            if ((state->phyLinkCaps & ENETPHY_LINK_CAP_1000) != 0U)
            {
                EnetPhy_rmwReg(hPhy, PHY_GIGCR, GIGCR_1000, 0U);
            }
        }

        state->needsManualCfg = false;
    }

    EnetPhy_capToMode(state->linkCaps, &state->speed, &state->duplexity);

    ENETTRACE_DBG("PHY %u: requested mode: %s\r\n",
                  hPhy->addr, EnetPhy_getModeString(state->speed, state->duplexity));
}

static void EnetPhy_loopbackState(EnetPhy_Handle hPhy)
{
    bool loopback = false;
    int32_t status;
    uint16_t val;

    status = EnetPhy_readReg(hPhy, PHY_BMCR, &val);
    if (status == ENETPHY_SOK)
    {
        loopback = ((val & BMCR_LOOPBACK) != 0U);
    }
    else
    {
        ENETTRACE_ERR("PHY %u: Failed to read BMCR register: %d\r\n", hPhy->addr, status);
    }

    if (!loopback)
    {
        EnetPhy_setNextState(hPhy, ENETPHY_FSM_STATE_FOUND);
    }
}

static void EnetPhy_nwayStartState(EnetPhy_Handle hPhy)
{
    EnetPhy_State *state = &hPhy->state;
    uint16_t mode;
    uint16_t val;

    Enet_devAssert(!hPhy->phyCfg.isStrapped,
                   "PHY %u: unexpected state for strapped PHY\r\n", hPhy->addr);

    /* Wait for NWAY to start */
    EnetPhy_readReg(hPhy, PHY_BMCR, &mode);

    if ((mode & BMCR_ANRESTART) == 0U)
    {
        /* Flush pending latch bits */
        EnetPhy_readReg(hPhy, PHY_BMSR, &val);
        EnetPhy_setNextState(hPhy, ENETPHY_FSM_STATE_NWAY_WAIT);
    }
    else
    {
        if (state->timeout != 0U)
        {
            state->timeout--;
        }
        else
        {
            EnetPhy_phyTimeout(hPhy);
        }
    }
}

static void EnetPhy_nwayWaitState(EnetPhy_Handle hPhy)
{
    EnetPhy_State *state = &hPhy->state;
    uint32_t nwayCaps;
    uint16_t status;

    Enet_devAssert(!hPhy->phyCfg.isStrapped,
                   "PHY %u: unexpected state for strapped PHY\r\n", hPhy->addr);

    EnetPhy_readReg(hPhy, PHY_BMSR, &status);
    if (0U != (status & BMSR_ANCOMPLETE))
    {
        nwayCaps = EnetPhy_findCommonNwayCaps(hPhy);
        if (nwayCaps != 0U)
        {
            EnetPhy_capToMode(nwayCaps, &state->speed, &state->duplexity);

            ENETTRACE_DBG("PHY %u: negotiated mode: %s\r\n",
                          hPhy->addr, EnetPhy_getModeString(state->speed, state->duplexity));

            if ((status & BMSR_LINKSTS) != 0U)
            {
                EnetPhy_setNextState(hPhy, ENETPHY_FSM_STATE_LINKED);
            }
            else
            {
                EnetPhy_setNextState(hPhy, ENETPHY_FSM_STATE_LINK_WAIT);
            }
        }
    }
    else
    {
        if (state->timeout != 0U)
        {
            state->timeout--;
        }
        else
        {
            EnetPhy_phyTimeout(hPhy);
        }
    }
}

static void EnetPhy_linkWaitState(EnetPhy_Handle hPhy)
{
    EnetPhy_State *state = &hPhy->state;
    uint32_t nwayCaps;
    uint16_t status;

    EnetPhy_readReg(hPhy, PHY_BMSR, &status);

    if ((status & BMSR_LINKSTS) != 0U)
    {
        /* Populate FSM state now as most FSM states are bypassed for strapped PHYs */
        if (hPhy->phyCfg.isStrapped)
        {
            state->phyLinkCaps = EnetPhy_getLocalCaps(hPhy);
            nwayCaps = EnetPhy_findCommonNwayCaps(hPhy);
            if (nwayCaps != 0U)
            {
                EnetPhy_capToMode(nwayCaps, &state->speed, &state->duplexity);

                ENETTRACE_DBG("PHY %u: negotiated mode: %s\r\n",
                              hPhy->addr, EnetPhy_getModeString(state->speed, state->duplexity));
            }
        }

        EnetPhy_setNextState(hPhy, ENETPHY_FSM_STATE_LINKED);
    }
    else
    {
        if (state->timeout != 0U)
        {
            state->timeout--;
        }
        else
        {
            EnetPhy_phyTimeout(hPhy);
        }
    }
}

static void EnetPhy_linkedState(EnetPhy_Handle hPhy)
{
    bool linked;

    linked = EnetPhy_isPhyLinked(hPhy);

    /* Not linked */
    if (!linked)
    {
        if (hPhy->phyCfg.isStrapped)
        {
            EnetPhy_setNextState(hPhy, ENETPHY_FSM_STATE_LINK_WAIT);
        }
        else
        {
            EnetPhy_setNextState(hPhy, ENETPHY_FSM_STATE_FOUND);
        }
    }
}


static bool EnetPhy_isNwayCapable(EnetPhy_Handle hPhy)
{
    uint16_t val = 0U;

    /* Get the PHY Status */
    EnetPhy_readReg(hPhy, PHY_BMSR, &val);

    return ((val & BMSR_ANCAPABLE) != 0U);
}

static uint32_t EnetPhy_getLocalCaps(EnetPhy_Handle hPhy)
{
    uint32_t caps = 0U;
    uint16_t val  = 0U;

    /* Get 10/100 Mbps capabilities */
    EnetPhy_readReg(hPhy, PHY_BMSR, &val);
    if ((val & BMSR_100FD) != 0U)
    {
        caps |= ENETPHY_LINK_CAP_FD100;
    }

    if ((val & BMSR_100HD) != 0U)
    {
        caps |= ENETPHY_LINK_CAP_HD100;
    }

    if ((val & BMSR_10FD) != 0U)
    {
        caps |= ENETPHY_LINK_CAP_FD10;
    }

    if ((val & BMSR_10HD) != 0U)
    {
        caps |= ENETPHY_LINK_CAP_HD10;
    }

    /* Get extended (1 Gbps) capabilities if supported */
    if ((val & BMSR_GIGEXTSTS) != 0U)
    {
        EnetPhy_readReg(hPhy, PHY_GIGESR, &val);

        if ((val & GIGESR_1000FD) != 0U)
        {
            caps |= ENETPHY_LINK_CAP_FD1000;
        }

        if ((val & GIGESR_1000HD) != 0U)
        {
            caps |= ENETPHY_LINK_CAP_HD1000;
        }
    }

    return caps;
}

static uint32_t EnetPhy_findCommonCaps(EnetPhy_Handle hPhy)
{
    uint32_t localCaps = 0U;
    uint32_t partnerCaps = 0U;
    uint16_t val = 0U;

    /* Get local device capabilities */
    EnetPhy_readReg(hPhy, PHY_ANAR, &val);

    if ((val & ANAR_100FD) != 0U)
    {
        localCaps |= ENETPHY_LINK_CAP_FD100;
    }

    if ((val & ANAR_100HD) != 0U)
    {
        localCaps |= ENETPHY_LINK_CAP_HD100;
    }

    if ((val & ANAR_10FD) != 0U)
    {
        localCaps |= ENETPHY_LINK_CAP_FD10;
    }

    if ((val & ANAR_10HD) != 0U)
    {
        localCaps |= ENETPHY_LINK_CAP_HD10;
    }

    /* Get link partner capabilities */
    val = 0U;
    EnetPhy_readReg(hPhy, PHY_ANLPAR, &val);

    if ((val & ANLPAR_100FD) != 0U)
    {
        partnerCaps |= ENETPHY_LINK_CAP_FD100;
    }

    if ((val & ANLPAR_100HD) != 0U)
    {
        partnerCaps |= ENETPHY_LINK_CAP_HD100;
    }

    if ((val & ANLPAR_10FD) != 0U)
    {
        partnerCaps |= ENETPHY_LINK_CAP_FD10;
    }

    if ((val & ANLPAR_10HD) != 0U)
    {
        partnerCaps |= ENETPHY_LINK_CAP_HD10;
    }

    ENETTRACE_DBG("PHY %u: local caps: %s\r\n",
                  hPhy->addr, EnetPhy_getCapsString(localCaps));
    ENETTRACE_DBG("PHY %u: partner caps: %s\r\n",
                  hPhy->addr, EnetPhy_getCapsString(partnerCaps));
    ENETTRACE_DBG("PHY %u: common caps: %s\r\n",
                  hPhy->addr, EnetPhy_getCapsString(localCaps & partnerCaps));

    return (localCaps & partnerCaps);
}

static uint32_t EnetPhy_findCommon1000Caps(EnetPhy_Handle hPhy)
{
    uint32_t localCaps = 0U;
    uint32_t partnerCaps = 0U;
    uint16_t val;

    /* Get local device capabilities */
    EnetPhy_readReg(hPhy, PHY_GIGCR, &val);

    if ((val & GIGCR_1000FD) != 0U)
    {
        localCaps |= ENETPHY_LINK_CAP_FD1000;
    }

    if ((val & GIGCR_1000HD) != 0U)
    {
        localCaps |= ENETPHY_LINK_CAP_HD1000;
    }

    /* Get link partner capabilities */
    EnetPhy_readReg(hPhy, PHY_GIGSR, &val);

    if ((val & GIGSR_1000FD) != 0U)
    {
        partnerCaps |= ENETPHY_LINK_CAP_FD1000;
    }

    if ((val & GIGSR_1000FD) != 0U)
    {
        partnerCaps |= ENETPHY_LINK_CAP_HD1000;
    }

    ENETTRACE_DBG("PHY %u: local caps: %s\r\n",
                  hPhy->addr, EnetPhy_getCapsString(localCaps));
    ENETTRACE_DBG("PHY %u: partner caps: %s\r\n",
                  hPhy->addr, EnetPhy_getCapsString(partnerCaps));
    ENETTRACE_DBG("PHY %u: common caps: %s\r\n",
                  hPhy->addr, EnetPhy_getCapsString(localCaps & partnerCaps));

    return (localCaps & partnerCaps);
}

static uint32_t EnetPhy_findCommonNwayCaps(EnetPhy_Handle hPhy)
{
    EnetPhy_State *state = &hPhy->state;
    uint32_t nwayCaps;

    nwayCaps = EnetPhy_findCommonCaps(hPhy);

    /* Find common gigabit capabilities if supported by PHY */
    if ((state->phyLinkCaps & ENETPHY_LINK_CAP_1000) != 0U)
    {
        nwayCaps |= EnetPhy_findCommon1000Caps(hPhy);
    }

    ENETTRACE_DBG("PHY %u: common caps: %s\r\n",
                  hPhy->addr, EnetPhy_getCapsString(nwayCaps));

    if (nwayCaps != 0U)
    {
        /* Find the highest performance protocol from the common
         * capabilities of local device and link partner */
        nwayCaps = EnetPhy_findBestCap(nwayCaps);
    }
    else
    {
        /* Non-compiant PHY? Flag the negotiation error */
        ENETTRACE_ERR("PHY %u: no common caps found\r\n", hPhy->addr);
    }

    return nwayCaps;
}

static bool EnetPhy_isPhyLinked(EnetPhy_Handle hPhy)
{
    EnetPhy_MdioHandle hMdio = hPhy->hMdio;
    uint32_t phyGroup = hPhy->group;
    uint32_t phyAddr = hPhy->addr;
    bool isLinked = false;
    uint16_t val = 0U;
    int32_t status;

    /* Get PHY link status */
    if (hMdio->isLinked != NULL)
    {
        /* Get link status from MDIO driver (i.e. hardware assisted) */
        status = hMdio->isLinked(phyAddr, &isLinked, hPhy->mdioArgs);
        ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                         "PHY %u: Failed to get link status: %d\r\n", phyAddr, status);
    }
    else
    {
        /* Alternatively, BMSR[2] Link Status bit can be checked */
        status = hMdio->readC22(phyGroup, phyAddr, PHY_BMSR, &val, hPhy->mdioArgs);
        ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                         "PHY %u: Failed to read reg %u: %d\r\n", phyAddr, PHY_BMSR, status);
        if ((status == ENETPHY_SOK) &&
            ((val & BMSR_LINKSTS) != 0U))
        {
            isLinked = true;
        }
    }

    return isLinked;
}

static int32_t EnetPhy_bindDriver(EnetPhy_Handle hPhy)
{
    EnetPhyDrv_Handle hDrv;
    EnetPhy_Version version;
    bool match;
    bool macSupported;
    uint32_t i;
    int32_t status;

    hPhy->hDrv = NULL;

    status = EnetPhy_getId(hPhy, &version);
    if (status == ENETPHY_SOK)
    {
        for (i = 0U; i < gEnetPhyDrvTbl.numHandles; i++)
        {
            hDrv = gEnetPhyDrvTbl.hPhyDrvList[i];

            Enet_devAssert(hDrv != NULL,
                           "PHY %u: No PHY driver handle at index %u\r\n",
                           hPhy->addr, i);
            Enet_devAssert(hDrv->isPhyDevSupported != NULL,
                           "PHY %u: PHY driver '%s' doesn't provide isPhyDevSupported callback\r\n",
                           hPhy->addr, hDrv->name);

            ENETTRACE_DBG("PHY %u: OUI:%06x Model:%02x Ver:%02x <-> '%s'\r\n",
                          hPhy->addr,
                          version.oui, version.model, version.revision,
                          hDrv->name);

            /* Check if driver supports detected PHY device */
            match = hDrv->isPhyDevSupported(hPhy, &version);

            /* Check if driver supports the MAC mode */
            if (match)
            {
                macSupported = hDrv->isMacModeSupported(hPhy, hPhy->mii);
                if (!macSupported)
                {
                    ENETTRACE_WARN("PHY %u: '%s' doesn't support MAC mode %d\r\n",
                                   hPhy->addr, hDrv->name, hPhy->mii);
                }
            }

            /* Bind the device <-> driver now */
            if (match)
            {
                hPhy->hDrv = hDrv;
                break;
            }
        }

        ENETTRACE_INFO_IF(hPhy->hDrv != NULL,
                          "PHY %u: OUI:%06x Model:%02x Ver:%02x <-> '%s' : OK\r\n",
                          hPhy->addr,
                          version.oui, version.model, version.revision,
                          hDrv->name);
    }

    return (hPhy->hDrv != NULL) ? ENETPHY_SOK : ENETPHY_EFAIL;
}

static void EnetPhy_resetPhy(EnetPhy_Handle hPhy)
{
    EnetPhy_State *state = &hPhy->state;

    ENETTRACE_WARN_IF(hPhy->hDrv == NULL,
                      "PHY %u: not bound to a driver, can't be reset\r\n", hPhy->addr);

    /* PHY specific reset */
    if ((hPhy->hDrv != NULL) &&
        (hPhy->hDrv->reset != NULL))
    {
        hPhy->hDrv->reset(hPhy);

        if (hPhy->phyCfg.isStrapped)
        {
            /* Strapped PHY doesn't need either Auto-Negotiation or manual configuration */
            state->needsManualCfg = false;
            state->needsNwayCfg   = false;
        }
        else
        {
            /* Typically manual/auto-negotiation should be reconfigured after PHY reset.
             * Certain reset/restart implementations only reset the logic not registers,
             * but it's safer to assume reconfiguration is needed */
            state->needsManualCfg = true;
            state->needsNwayCfg   = true;
        }
    }
}

static uint32_t EnetPhy_getManualCaps(EnetPhy_Speed speed,
                                      EnetPhy_Duplexity duplexity)
{
    uint32_t caps = 0U;

    Enet_devAssert((speed != ENETPHY_SPEED_AUTO) &&
                   (duplexity != ENETPHY_DUPLEX_AUTO),
                   "Invalid speed/duplex for manual mode\r\n");

    switch (speed)
    {
        case ENETPHY_SPEED_10MBIT:
            caps = (duplexity == ENETPHY_DUPLEX_FULL) ?
                   ENETPHY_LINK_CAP_FD10 :
                   ENETPHY_LINK_CAP_HD10;
            break;

        case ENETPHY_SPEED_100MBIT:
            caps = (duplexity == ENETPHY_DUPLEX_FULL) ?
                   ENETPHY_LINK_CAP_FD100 :
                   ENETPHY_LINK_CAP_HD100;
            break;

        case ENETPHY_SPEED_1GBIT:
            caps = (duplexity == ENETPHY_DUPLEX_FULL) ?
                   ENETPHY_LINK_CAP_FD1000 :
                   ENETPHY_LINK_CAP_HD1000;
            break;

        default:
            break;
    }

    return caps;
}

static uint32_t EnetPhy_findBestCap(uint32_t caps)
{
    uint32_t i;

    for (i = ENETPHY_LINK_CAP_FD1000; i >= ENETPHY_LINK_CAP_HD10; i >>= 1U)
    {
        if ((caps & i) != 0U)
        {
            break;
        }
    }

    return i;
}

static void EnetPhy_capToMode(uint32_t caps,
                              EnetPhy_Speed *speed,
                              EnetPhy_Duplexity *duplexity)
{
    if (caps & ENETPHY_LINK_CAP_FD1000)
    {
        *speed  = ENETPHY_SPEED_1GBIT;
        *duplexity = ENETPHY_DUPLEX_FULL;
    }
    else if (caps & ENETPHY_LINK_CAP_HD1000)
    {
        *speed  = ENETPHY_SPEED_1GBIT;
        *duplexity = ENETPHY_DUPLEX_HALF;
    }
    else if (caps & ENETPHY_LINK_CAP_FD100)
    {
        *speed  = ENETPHY_SPEED_100MBIT;
        *duplexity = ENETPHY_DUPLEX_FULL;
    }
    else if (caps & ENETPHY_LINK_CAP_HD100)
    {
        *speed  = ENETPHY_SPEED_100MBIT;
        *duplexity = ENETPHY_DUPLEX_HALF;
    }
    else if (caps & ENETPHY_LINK_CAP_FD10)
    {
        *speed  = ENETPHY_SPEED_10MBIT;
        *duplexity = ENETPHY_DUPLEX_FULL;
    }
    else
    {
        *speed  = ENETPHY_SPEED_10MBIT;
        *duplexity = ENETPHY_DUPLEX_HALF;
    }
}

static void EnetPhy_showLinkPartnerCompat(EnetPhy_Handle hPhy,
                                          EnetPhy_Speed speed,
                                          EnetPhy_Duplexity duplexity)
{
    switch (speed)
    {
        case ENETPHY_SPEED_10MBIT:
        case ENETPHY_SPEED_100MBIT:
            if (duplexity == ENETPHY_DUPLEX_HALF)
            {
                ENETTRACE_DBG("PHY %u: recommended link partner config: "
                              "manual mode at %s or auto-negotiation\r\n",
                              hPhy->addr, EnetPhy_getModeString(speed, duplexity));
            }
            else
            {
                ENETTRACE_DBG("PHY %u: recommended link partner config: "
                              "manual mode at %s\r\n",
                              hPhy->addr, EnetPhy_getModeString(speed, duplexity));
            }

            break;

        case ENETPHY_SPEED_1GBIT:
            ENETTRACE_DBG("PHY %u: recommended link partner config: "
                          "manual mode at %s\r\n",
                          hPhy->addr, EnetPhy_getModeString(speed, duplexity));
            break;

        default:
            break;
    }
}
