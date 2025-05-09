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
 * \file  cpsw_priv.h
 *
 * \brief This file contains the private type definitions and helper macros for
 *        the CPSW peripheral.
 */

#ifndef CPSW_PRIV_H_
#define CPSW_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <priv/core/enet_base_priv.h>
#include <priv/mod/cpsw_ale_priv.h>
#include <priv/mod/cpsw_cpts_priv.h>
#include <priv/mod/cpsw_hostport_priv.h>
#include <priv/mod/cpsw_macport_priv.h>
#include <priv/mod/mdio_priv.h>
#include <priv/mod/cpsw_stats_priv.h>
#include <priv/core/enet_rm_priv.h>
#include <include/core/enet_per.h>
#include <include/core/enet_dma.h>
#include <include/per/cpsw.h>
#include <include/phy/enetphy.h>
#if ENET_CFG_IS_ON(CPSW_EST)
#include <priv/per/cpsw_est_priv.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief InterVLAN feature mask. */
#define CPSW_FEATURE_INTERVLAN                (ENET_BIT(1U))

/*! \brief EST feature mask. */
#define CPSW_FEATURE_EST                      (ENET_BIT(2U))

/*! \brief Cut-thru feature mask. */
#define CPSW_FEATURE_CUTTHRU                  (ENET_BIT(3U))

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

typedef struct SemaphoreP_enetOsal_s SemaphoreP_enetOsal;

typedef struct HwiP_enetOsal_s HwiP_enetOsal;

/*!
 * \brief MDIO status change (MDIO_LINKINT) context.
 */
typedef struct Cpsw_MdioLinkIntCtx_s
{
    /*! Alive PHYs, updated upon MDIO_LINKINT interrupt */
    uint32_t aliveMask;

    /*! Linked PHYs, updated upon MDIO_LINKINT interrupt */
    uint32_t linkedMask;

    /*! Mask of PHYs with status change poll enabled */
    uint32_t pollEnableMask;

    /*! Link state change callback function pointer */
    Cpsw_MdioLinkStateChangeCb linkStateChangeCb;

    /*! Application data to be passed to the link state change callback */
    void *linkStateChangeCbArg;
} Cpsw_MdioLinkIntCtx;

/*!
 * \brief Port link state (link up/down, tick enabled)
 */
typedef struct Cpsw_PortLinkState_s
{
    /*! PHY address */
    uint32_t phyAddr;

    /*! Whether link port has been opened */
    bool isOpen;

    /*! Port link up/down state as reported by driver's FSM */
    bool isLinkUp;

    /*! Whether MDIO state change polling is enabled on this port's PHY */
    bool isPollEnabled;

    /*! Whether PHY periodic tick is enabled on the port link's PHY */
    bool isTickEnabled;
} Cpsw_PortLinkState;

/*!
 * \brief CPSW driver object
 */
typedef struct Cpsw_Obj_s
{
    /*! Peripheral name */
    const char *name;

    /*! Peripheral type */
    Enet_Type enetType;

    /*! Peripheral instance id */
    uint32_t instId;

    /*! Peripheral initialization magic */
    Enet_Magic magic;

    /*! Peripheral's physical address. Used for peripherals that have registers
     *  that are not part of any module (i.e. peripherals that have a wrapper
     *  subsystem).
     *  It can be set to 0 for peripherals that  don't have have additional
     *  registers other than those of their modules. */
    uint64_t physAddr;

    /*! Peripheral's virtual address */
    void *virtAddr;

    /*! Peripheral's second physical address, if needed */
    uint64_t physAddr2;

    /*! Peripheral's second virtual address, if needed */
    void *virtAddr2;

    /*! Peripheral features */
    uint32_t features;

    /*! Peripheral applicable errata */
    uint32_t errata;

    /*! Host port module */
    CpswHostPort_Obj hostPortObj;

    /*! MAC port objects */
    CpswMacPort_Obj *macPortObj;

    /*! Number of MAC port ojects in #macPortObj */
    uint32_t macPortNum;

    /*! ALE object */
    CpswAle_Obj aleObj;

    /*! CPTS object */
    CpswCpts_Obj cptsObj;

    /*! MDIO object */
    Mdio_Obj mdioObj;

    /*! Network statistics object */
    CpswStats_Obj statsObj;

    /*! RM object */
    EnetRm_Obj rmObj;

    /*! DMA handle */
    EnetDma_Handle hDma;

    /*! CPSW DMA Rx Reserved flow handle*/
    EnetDma_RxChHandle hRxRsvdFlow;

    /*! CPSW DMA resource information*/
    Enet_dmaResInfo dmaResInfo;

    /*! CPSW DMA Rx Reserved flow Id */
    uint32_t rsvdFlowId;

    /*! PHY handles */
    EnetPhy_Handle hPhy[CPSW_MAC_PORT_NUM];

    /*! Core on which Cpsw_Open() is executed */
    uint32_t selfCoreId;

    /*! PHY link up/down state */
    Cpsw_PortLinkState portLinkState[CPSW_MAC_PORT_NUM];

    /*! Statistics interrupt handle */
    HwiP_enetOsal * hStatsIntr;

    /*! MDIO interrupt handle */
    HwiP_enetOsal * hMdioIntr;

    /*! CPTS interrupt handle */
    HwiP_enetOsal * hCptsIntr;

    /*! MDIO link state change interrupt context */
    Cpsw_MdioLinkIntCtx mdioLinkIntCtx;

    /*! Port link status change callback function pointer */
    Cpsw_PortLinkStatusChangeCb portLinkStatusChangeCb;

    /*! Application data to be passed to the port link status change callback */
    void *portLinkStatusChangeCbArg;

    /*! Maximum of MTUs for TX Priority 0 to 7 */
    uint32_t maxPerPrioMtu;

#if ENET_CFG_IS_ON(CPSW_EST)
    /*! CPTS RFT clock frequency, needed to compute ESTF length value. */
    CpswCpts_RftClkFreq cptsRftClkFreq;

    /*! Per-port EST status */
    Cpsw_EstState estState[CPSW_MAC_PORT_NUM];
#endif

    /*! Disable Enet LLD PHY driver - Disables use on PHY driver inside the
     *  Enet LLD. All PHY functionality including PHY state machine is bypassed
     *  Application will use this mode if ethernet PHY is managed outside the Enet LLD
     *  Application is responsible for PHY management. Application can register
     *  with Enet LLD to get mdioLinkStateChangeCb callback.
     *  Application _must_ use Enet LLD IOCTLs to access MDIO as MDIO ownership
     *  is still with Enet LLD and there should not be multiple masters for the
     *  MDIO peripheral
     */
    bool disablePhyDriver;

    /* Saving CpswCfg before resetting */
    Cpsw_Cfg context;

    /*! Main, API-level Lock */
    SemaphoreP_enetOsal * lock;
} Cpsw_Obj;

/*!
 * \brief CPSW peripheral handle.
 */
typedef Cpsw_Obj *Cpsw_Handle;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize Enet LLD.
 *
 * One-time initialization of the Enet LLD driver.  This function initializes
 * the OSAL and utils infrastructure that the driver requires for handling
 * multiple peripherals as well as logging and tracing.
 *
 * The Enet LLD provides a default OSAL implementation which is based on PDK
 * OSAL library if ENET_CFG_HAS_DEFAULT_OSAL config flag is enabled.  The
 * default OSAL implementation can be used if the caller passes a NULL
 * \p osalCfg.
 *
 * Similarly, the Enet LLD provides a default utils implementation if
 * ENET_CFG_HAS_DEFAULT_UTILS config flag is set.  The default utils
 * implementation can be used if the caller passes a NULL \p utilsCfg.
 *
 * \param osalCfg   OSAL configuration parameters
 * \param utilsCfg  Utils configuration parameters
 */

void Cpsw_init(const EnetUtils_Cfg *utilsCfg);

/*!
 * \brief De-initialize Enet LLD.
 *
 * One-time de-initialization of the Enet LLD driver.  This function clears
 * the OSAL and utils config parameters passed during Enet_init().
 *
 * It's expected to be called once all Ethernet peripherals have been closed
 * via Enet_close().
 */

void Cpsw_deinit(void);

/*!
 * \brief Initialize CPSW peripheral's configuration parameters.
 *
 * Initializes the configuration parameter of the CPSW peripheral.
 *
 * \param cpswCfg   Configuration parameters to be initialized.  The config
 *                  is of type #Cpsw_Cfg.
 */
void Cpsw_initCfg(Cpsw_Cfg *cpswCfg);

/*!
 * \brief Open and initialize the CPSW Peripheral.
 *
 * Opens and initializes the CPSW peripheral with the configuration parameters
 * provided by the caller.
 *
 * \param hPer      Enet Peripheral handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters to be initialized.  The config
 *                  is of type #Cpsw_Cfg.
 * \param cfgSize   Size of the configuration parameters.  It must be the size
 *                  of #Cpsw_Cfg config structure.
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t Cpsw_open(uint32_t hEnet,
                  Enet_Type enetType,
                  uint32_t instId,
                  const Cpsw_Cfg *cpswCfg);

/*!
 * \brief Rejoin a running CPSW peripheral.
 *
 * Reopens the CPSW peripheral, but doesn't perform any hardware initialization.
 * This function is expected to be called to attach to a running peripheral.
 *
 * This is an optional function and could be set to NULL if the peripheral
 * doesn't implement it.
 *
 * \param hPer      Enet Peripheral handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t Cpsw_rejoin(uint32_t hEnet,
                    Enet_Type enetType,
                    uint32_t instId);

/*!
 * \brief Close the CPSW peripheral.
 *
 * Closes the CPSW peripheral.
 *
 * \param hPer        Enet Peripheral handle
 */
void Cpsw_close(uint32_t hEnet);

/*!
 * \brief Issue an operation on the CPSW peripheral.
 *
 * Issues a control operation on the CPSW peripheral.
 *
 * \param hPer         Enet Peripheral handle
 * \param cmd          IOCTL command Id
 * \param prms         IOCTL parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t Cpsw_ioctl(uint32_t hEnet,
                   uint32_t cmd,
                   Enet_IoctlPrms *prms);

/*!
 * \brief Poll for Ethernet events.
 *
 * The CPSW peripheral currently doesn't provide polling support.
 *
 * \param hPer         Enet Peripheral handle
 * \param evt          Event type
 * \param arg          Pointer to the poll argument. This is specific to the
 *                     poll event type
 * \param argSize      Size of \p arg
 */
void Cpsw_poll(uint32_t hEnet,
               Enet_Event evt,
               const void *arg,
               uint32_t argSize);

/*!
 * \brief Run periodic tick on the CPSW peripheral.
 *
 * Run PHY periodic tick on the CPSW peripheral.  The peripheral driver in
 * turn runs the periodic tick operation on all opened PHYs.
 *
 * \param hPer        Enet Peripheral handle
 */
void Cpsw_periodicTick(uint32_t hEnet);

/*!
 * \brief Saves and Close the CPSW peripheral.
 *
 * Closes the CPSW peripheral.
 *
 * \param hPer        Enet Peripheral handle
 */
void Cpsw_saveCtxt(Cpsw_Handle hCpsw);

/*!
 * \brief Restoes and Open the CPSW Peripheral.
 *
 * Opens and initializes the CPSW peripheral with the configuration parameters
 * provided by the caller.
 *
 * \param hPer      Enet Peripheral handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t Cpsw_restoreCtxt(Cpsw_Handle hCpsw,
                         Enet_Type enetType,
                         uint32_t instId);

int32_t Cpsw_getHandleInfo(uint32_t hEnet,
                           Enet_Type *enetType,
                           uint32_t *instId);

Cpsw_Handle Cpsw_getHandle(uint32_t hEnet);

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

#endif /* CPSW_PRIV_H_ */
