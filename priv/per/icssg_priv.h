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
 * \file  icssg_priv.h
 *
 * \brief This file contains the private type definitions and helper macros for
 *        the ICSSG peripheral.
 */

#ifndef ICSSG_PRIV_H_
#define ICSSG_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/pruicss.h>
#include <priv/core/enet_base_priv.h>
#include <priv/mod/mdio_priv.h>
#include <priv/mod/icssg_timesync_priv.h>
#include <priv/mod/icssg_stats_priv.h>
#include <priv/mod/icssg_tas_priv.h>
#include <enet.h>
#include <include/core/enet_per.h>
#include <include/core/enet_types.h>
#include <include/mod/mdio.h>
#include <include/phy/enetphy.h>
#include <include/core/enet_mod_phy.h>
#include <priv/core/enet_rm_priv.h>


#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
/*! \brief Helper macro to create private IOCTL commands for CPSW MAC port module. */
#define ICSSG_PRIVATE_IOCTL(x)                 (ENET_IOCTL_TYPE_PRIVATE |  \
                                                ENET_IOCTL_PER_BASE |      \
                                                ENET_IOCTL_PER_ICSSG |     \
                                                ENET_IOCTL_MIN(x))


/*! \brief PRU ICSS subsystem instance 0 */
#define ICSSG_PRUSS_ID_0                          (0U)

/*! \brief PRU ICSS subsystem instance 1 */
#define ICSSG_PRUSS_ID_1                          (1U)

/*! \brief PRU ICSS subsystem instance 2 */
#define ICSSG_PRUSS_ID_2                          (2U)

/*! \brief Cache alignment used for IOCTL command structure */
#define ICSSG_CACHELINE_ALIGNMENT                 (64U)

/*! \brief ICSS switch QOS level to HOST_BUFFER_POOL_NUM Factor
 *
 * In Switch mode to support max QoS level of 8
 * number of pools should be set to 8 * number of ports
 * which is 3 (two external ports and 1 host port)
 */
#define ICSSG_SWITCH_HOST_BUFFER_POOL_NUM_QOS_MULTIPLE    (3U)

/*! \brief ICSS switch QOS level to HOST_BUFFER_POOL_NUM Factor
 *
 * In Dualmac mode to support max QoS level of 8
 * number of pools should be set to 8 * 1
 */
#define ICSSG_DUALMAC_HOST_BUFFER_POOL_NUM_QOS_MULTIPLE   (1U)

/*! \brief Number of Host Buffer pools for dual mac mode
 *
 * In DUAL_MAC mode to support max QoS level of 8
 * number of pools should be set to 8
 */
#define ICSSG_DUALMAC_GET_HOST_BUFFER_POOL_NUM(qos)     ((qos) * (ICSSG_DUALMAC_HOST_BUFFER_POOL_NUM_QOS_MULTIPLE))

/*! \brief Number of Host Buffer pools for switch mode
 *
 * In Switch mode to support max QoS level of 8
 * number of pools should be set to 8 * number of ports
 * which is 3 (two external ports and 1 host port)
 */
#define ICSSG_SWITCH_GET_HOST_BUFFER_POOL_NUM(qos)     ((qos) * (ICSSG_SWITCH_HOST_BUFFER_POOL_NUM_QOS_MULTIPLE))


/*! \brief Maximum number of QoS levels supported */
#define ICSSG_QOS_MAX                                  (ENET_PRI_NUM)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
/*!
 * \brief ICSSG Private IOCTL commands.Invoked within Enet LLD and not by application
 */
typedef enum Icssg_PrivIoctl_e
{
    /*!
     * \brief Register IOCTL handler
     *
     * IOCTL params:
     * -  inArgs: #Enet_IoctlRegisterHandlerInArgs
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    ICSSG_MACPORT_IOCTL_REGISTER_HANDLER = ICSSG_PRIVATE_IOCTL(0U),

} Icssg_PrivIoctl;

/*!
 * \brief Icssg event callback info structure
 */
typedef struct Icssg_evtCbInfo_s
{
    /*! Event callback function for command complete response event */
    Enet_EventCallback evtCb;

    /*! Event callback function arguments */
    void *evtCbArgs;
} Icssg_evtCbInfo;

/*!
 * \brief ICSS_PRU object.
 *
 * This structure represents the ICSS_PRU subsystem irrespective of the Ethernet
 * specific abstractions of the driver.  For instance, the two MAC ports in ICSSG
 * Dual-MAC mode are abstracted logically as separate peripherals in spite of
 * both running on the same ICSSPRU subsystem.
 *
 * In ICSSG Dual-MAC mode, each of the two #Icssg_Obj structures will have a
 * pointer to the same #Icssg_Pruss object.
 *
 * In Switch mode, there is a single #Icssg_Obj per PRUSS, also indicated via
 * pointer to the corresponding #Icssg_Pruss object.
 *
 * The #Icssg_Obj to #Icssg_Pruss binding is done by EnetSoc layer.
 */
typedef struct Icssg_Pruss_s
{
    /*! PRUICSS Obj index in gPruIcssConfig
     * Expected to carry the name shown in PRUICSS syscfg-gui */
    uint32_t name;

    /*! Handle to PRUICSS driver */
    PRUICSS_Handle hPruss;

    /*! PRUICSS instance number */
    uint32_t instance;

    /*! Whether the PRUICSSG has been initialized or not */
    bool initialized;

    /*! Whether IEP0 is in use or not */
    bool iep0InUse;

    /*! PRUSS instance lock to be used to protect any operations that involve
     *  any fields of this structure, i.e. using PRUSS driver.
     *  This lock is to be initialized by SoC layer. */
    void *lock;
} Icssg_Pruss;

/*!
 * \brief ICSSG firmware.
 *
 * Container structure for the ICSSG firmware blobs for PRU, RTU and TX PRU.
 */
typedef struct Icssg_Fw_s
{
    /*! Pointer to PRU firmware header */
    const uint32_t *pru;

    /*! Size of PRU firmware header */
    uint32_t pruSize;

    /*! Pointer to RTU firmware header */
    const uint32_t *rtu;

    /*! Size of RTU firmware header */
    uint32_t rtuSize;

    /*! Pointer to TXPRU firmware header */
    const uint32_t *txpru;

    /*! Size of TX PRU firmware header */
    uint32_t txpruSize;
} Icssg_Fw;

/*!
 * \brief IOCTL command structure used to communicate with ICSSG.
 */
typedef struct Icssg_IoctlCmd_s
{
    /*! Command parameter */
    uint8_t param;

    /*! Sequence number */
    uint8_t seqNum;

    /*! Command type */
    uint8_t type;

    /*! Command header */
    uint8_t header;

    /*! Spare data. Used for commands that take additional arguments */
    uint32_t spare[3];
} __attribute__((packed)) Icssg_IoctlCmd;

/*!
 * \brief IOCTL command response structure used to communicate with ICSSG.
 */
typedef struct Icssg_IoctlCmdResp_s
{
    /*! Status for IOCTL command */
    uint8_t status;

    /*! Sequence number of the command request. Returned to application in response */
    uint8_t seqNum;

    /*! Number of bytes in the respParams field, this is optional */
    uint32_t paramsLen;

    /*! Optional response parameters */
    uint32_t params[3U];
} Icssg_IoctlCmdResp;

/*!
 * \brief MDIO status change (MDIO_LINKINT) context.
 */
typedef struct Icssg_MdioLinkIntCtx_s
{
    /*! Alive PHYs, updated upon MDIO_LINKINT interrupt */
    uint32_t aliveMask;

    /*! Linked PHYs, updated upon MDIO_LINKINT interrupt */
    uint32_t linkedMask;

    /*! Mask of PHYs with status change poll enabled */
        uint32_t pollEnableMask;

    /*! Link state change callback function pointer */
    Icssg_MdioLinkStateChangeCb linkStateChangeCb;

    /*! Application data to be passed to the link state change callback */
    void *linkStateChangeCbArg;

    /*! PRU Interrupt Event Number */
    uint32_t pruEvtNum[ICSSG_MAC_PORT_MAX];
    /*! MDIO interrupt handle */
    void *hMdioIntr;
    const PRUICSS_IntcInitData  *prussIntcInitData;
} Icssg_MdioLinkIntCtx;

/*!
 * \brief Port link state (link up/down, tick enabled)
 */

/*!
 * \brief Icssg per object.
 */
typedef struct Icssg_Obj_s
{
    /*! EnetMod must be the first member */
    EnetPer_Obj enetPer;

    /*! PRUSS instance. SoC layer binds this ICSSG object to the corresponding PRUSS. */
    Icssg_Pruss *pruss;

    /*! ICSSG firmware configuration: image addresses and sizes.
     *  - Switch peripheral (#ENET_ICSSG_SWITCH), application must populate all
     *    firmwares entries of this array.
     *  - Dual-MAC peripheral (#ENET_ICSSG_DUALMAC), application must populate
     *    only the first firmware entry. */
    Icssg_Fw fw[ICSSG_MAC_PORT_MAX];

    /*! Asycnronuous IOCTL sequence number */
    uint8_t asyncIoctlSeqNum;

    /*! Asycnronuous IOCTL type */
    uint32_t asyncIoctlType;

    /*! Event callback information object for async command resp. callback */
    Icssg_evtCbInfo asyncCmdRespCbEvtInfo;

    /*! Event callback information object for TX timestamp event callback */
    Icssg_evtCbInfo txTsCbEvtInfo;

    /*! Resource Manager object */
    EnetRm_Obj rmObj;

    /*! Resource Manager handle */
    EnetMod_Handle hRm;

    /*! Core on which Icssg_Open() is executed */
    uint32_t selfCoreId;

    /*! DMA handle */
    EnetDma_Handle hDma;

    /*! Number of required UDMA RX channels */
    uint32_t numRxCh;

    /*! DMA Rx Reserved flow handle */
    EnetDma_RxChHandle hRxRsvdFlow[ICSSG_MAC_PORT_MAX];

    /*! DMA resource information */
    Enet_dmaResInfo dmaResInfo[ICSSG_MAC_PORT_MAX];

    /*! DMA Rx Reserved flow Id */
    uint32_t rsvdFlowId[ICSSG_MAC_PORT_MAX];

    /*! MDIO object */
    Mdio_Obj mdioObj;

    /*! MDIO handle */
    EnetMod_Handle hMdio;

    /*! PHY handles */
    EnetPhy_Handle hPhy[ICSSG_MAC_PORT_MAX];

    /*! TimesSync object */
    IcssgTimeSync_Obj timeSyncObj;

    /*! TimesSync handle */
    EnetMod_Handle hTimeSync;

    /*! Stats object */
    IcssgStats_Obj statsObj;

    /*! Stats handle */
    EnetMod_Handle hStats;

    /*! Tas object */
    IcssgTas_Obj tasObj[ICSSG_MAC_PORT_MAX];

    /*! Tas handle */
    EnetMod_Handle hTas[ICSSG_MAC_PORT_MAX];

    /*! IOCTL command */
    Icssg_IoctlCmd cmd __attribute__ ((aligned(ICSSG_CACHELINE_ALIGNMENT)));

    /*! Cycle time in nanoseconds */
    uint32_t cycleTimeNs;

    Icssg_MdioLinkIntCtx mdioLinkIntCtx;

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

    /*!  Max number of QoS Level supported. Used to determine number of hostBufferPoolNum */
    uint32_t qosLevels;

    /*! Whether premption Queue is enabled or not  */
    uint32_t isPremQueEnable;

    /*! Clock type in firmware */
    IcssgTimeSync_ClkType clockTypeFw;
} Icssg_Obj;

/*!
 * \brief MAC port module handle.
 */
typedef Icssg_Obj *Icssg_Handle;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize ICSSG peripheral's configuration parameters.
 *
 * Initializes the configuration parameter of the ICSSG peripheral.
 *
 * \param hPer      Enet Peripheral handle
 * \param enetType  Enet Peripheral type
 * \param cfg       Configuration parameters to be initialized.  The config
 *                  is of type #Icssg_Cfg.
 * \param cfgSize   Size of the configuration parameters.  It must be the size
 *                  of #Icssg_Cfg config structure.
 */
void Icssg_initCfg(EnetPer_Handle hPer,
                   Enet_Type enetType,
                   void *cfg,
                   uint32_t cfgSize);

/*!
 * \brief Open and initialize the ICSSG Peripheral.
 *
 * Opens and initializes the ICSSG peripheral with the configuration parameters
 * provided by the caller.
 *
 * \param hPer      Enet Peripheral handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters to be initialized.  The config
 *                  is of type #Icssg_Cfg.
 * \param cfgSize   Size of the configuration parameters.  It must be the size
 *                  of #Icssg_Cfg config structure.
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t Icssg_open(EnetPer_Handle hPer,
                   Enet_Type enetType,
                   uint32_t instId,
                   const void *cfg,
                   uint32_t cfgSize);

/*!
 * \brief Rejoin a running ICSSG peripheral.
 *
 * This operation is not supported by the ICSSG peripheral.  Calling this
 * function will return #ENET_ENOTSUPPORTED.
 *
 * \param hPer      Enet Peripheral handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \retval ENET_ENOTSUPPORTED
 */
int32_t Icssg_rejoin(EnetPer_Handle hPer,
                     Enet_Type enetType,
                     uint32_t instId);

/*!
 * \brief Issue an operation on the ICSSG peripheral.
 *
 * Issues a control operation on the ICSSG peripheral.
 *
 * \param hPer         Enet Peripheral handle
 * \param cmd          IOCTL command Id
 * \param prms         IOCTL parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t Icssg_ioctl(EnetPer_Handle hPer,
                    uint32_t cmd,
                    Enet_IoctlPrms *prms);

/*!
 * \brief Poll for Ethernet events.
 *
 * Unblocking poll for the events specified in \p evt. ICSSG uses this
 * function to poll for completion of asynchronous IOCTLs.
 *
 * \param hPer         Enet Peripheral handle
 * \param evt          Event type
 * \param arg          Pointer to the poll argument. This is specific to the
 *                     poll event type
 * \param argSize      Size of \p arg
 */
void Icssg_poll(EnetPer_Handle hPer,
                Enet_Event evt,
                const void *arg,
                uint32_t argSize);

/*!
 * \brief Converts ICSSG timestamp to nanoseconds.
 *
 * ICSSG timestamp encodes IEP count low/high and rollover count as bit fields
 * in the 64-bit value returned by ICSSG.  This value needs to be converted
 * to nanoseconds before application can consume it.
 *
 * \param hPer         Enet Peripheral handle
 * \param ts           Timestamp value, definition is peripheral specific
 *
 * \return Nanoseconds value.
 */
uint64_t Icssg_convertTs(EnetPer_Handle hPer,
                         uint64_t ts);

/*!
 * \brief Run periodic tick on the ICSSG peripheral.
 *
 * Run PHY periodic tick on the ICSSG peripheral.  The peripheral driver in
 * turn runs the periodic tick operation on all opened PHYs.
 *
 * \param hPer        Enet Peripheral handle
 */
void Icssg_periodicTick(EnetPer_Handle hPer);

void Icssg_registerEventCb(EnetPer_Handle hPer,
                           Enet_Event evt,
                           uint32_t evtNum,
                           Enet_EventCallback evtCb,
                           void *evtCbArgs);

void Icssg_unregisterEventCb(EnetPer_Handle hPer,
                             Enet_Event evt,
                             uint32_t evtNum);

/*!
 * \brief Close the ICSSG peripheral.
 *
 * Closes the ICSSG peripheral.
 *
 * \param hPer        Enet Peripheral handle
 */
void Icssg_close(EnetPer_Handle hPer);

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

#endif /* ICSSG_PRIV_H_ */
