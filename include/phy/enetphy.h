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
 * \file  enetphy.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Ethernet PHY interface.
 */

/*!
 * \ingroup  DRV_ENET_MODULE
 * \defgroup DRV_ENETPHY Ethernet PHY Driver
 *
 * The Ethernet PHY driver supports auto-negotiation and manual modes.  The
 * PHY driver uses an MDIO abstraction to perform register reads and writes.
 * The MDIO abstraction can be implemented using Enet LLD's \ref ENET_MOD_MDIO
 * API or any other MDIO driver.
 *
 * @{
 */

/*!
 * \ingroup  DRV_ENETPHY
 * \defgroup ENETPHY_GENERIC_PHY Generic PHY
 *
 * The ENETPHY driver provides a default implementation of a generic PHY which
 * performs basic configuration limited to the IEEE standard registers.
 * This generic PHY driver doesn't take any configuration structure.
 */

#ifndef ENETPHY_H_
#define ENETPHY_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Check if PHY address is valid (0 - 31). */
#define ENETPHY_IS_ADDR_VALID(addr)           ((addr) <= 31U)

/*! \brief Macro to set bit at given bit position. */
#define ENETPHY_BIT(n)                        (1U << (n))

/*! \brief Macro to check if bit at given bit position is set. */
#define ENETPHY_IS_BIT_SET(val, n)            (((val) & ENETPHY_BIT(n)) != 0U)

/*! \brief Macro to get the size of an array. */
#define ENETPHY_ARRAYSIZE(x)                  (sizeof(x) / sizeof(x[0]))

/*!
 * \anchor EnetPhy_ErrorCodes
 * \name   Ethernet PHY driver error codes
 *
 * Error codes returned by the Ethernet PHY driver APIs.
 *
 * @{
 */

/* Ethernet PHY driver error codes are same as CSL's to maintain consistency */

/*! \brief Success. */
#define ENETPHY_SOK                           (CSL_PASS)

/*! \brief Generic failure error condition (typically caused by hardware). */
#define ENETPHY_EFAIL                         (CSL_EFAIL)

/*! \brief Bad arguments (i.e. NULL pointer). */
#define ENETPHY_EBADARGS                      (CSL_EBADARGS)

/*! \brief Invalid parameters (i.e. value out-of-range). */
#define ENETPHY_EINVALIDPARAMS                (CSL_EINVALID_PARAMS)

/*! \brief Time out while waiting for a given condition to happen. */
#define ENETPHY_ETIMEOUT                      (CSL_ETIMEOUT)

/*! \brief Allocation failure. */
#define ENETPHY_EALLOC                        (CSL_EALLOC)

/*! \brief Operation not permitted. */
#define ENETPHY_EPERM                         (CSL_EALLOC - 4)

/*! \brief Operation not supported. */
#define ENETPHY_ENOTSUPPORTED                 (CSL_EALLOC - 5)

/*! @} */

/*!
 * \anchor EnetPhy_LinkCaps
 * \name   Ethernet PHY link capability masks
 *
 * Error codes returned by the Ethernet PHY driver APIs.
 *
 * @{
 */

/*! \brief 10-Mbps, half-duplex capability mask. */
#define ENETPHY_LINK_CAP_HD10                 ENETPHY_BIT(1)

/*! \brief 10-Mbps, full-duplex capability mask. */
#define ENETPHY_LINK_CAP_FD10                 ENETPHY_BIT(2)

/*! \brief 100-Mbps, half-duplex capability mask. */
#define ENETPHY_LINK_CAP_HD100                ENETPHY_BIT(3)

/*! \brief 100-Mbps, full-duplex capability mask. */
#define ENETPHY_LINK_CAP_FD100                ENETPHY_BIT(4)

/*! \brief 1-Gbps, half-duplex capability mask. */
#define ENETPHY_LINK_CAP_HD1000               ENETPHY_BIT(5)

/*! \brief 1-Gbps, full-duplex capability mask. */
#define ENETPHY_LINK_CAP_FD1000               ENETPHY_BIT(6)

/*! \brief 10-Mbps, full and half-duplex capability mask. */
#define ENETPHY_LINK_CAP_10                   (ENETPHY_LINK_CAP_HD10 | \
                                               ENETPHY_LINK_CAP_FD10)

/*! \brief 100-Mbps, full and half-duplex capability mask. */
#define ENETPHY_LINK_CAP_100                  (ENETPHY_LINK_CAP_HD100 | \
                                               ENETPHY_LINK_CAP_FD100)

/*! \brief 1-Gbps, full and half-duplex capability mask. */
#define ENETPHY_LINK_CAP_1000                 (ENETPHY_LINK_CAP_HD1000 | \
                                               ENETPHY_LINK_CAP_FD1000)

/*! \brief Auto-negotiation mask with all duplexity and speed values set. */
#define ENETPHY_LINK_CAP_ALL                  (ENETPHY_LINK_CAP_HD10 |   \
                                               ENETPHY_LINK_CAP_FD10 |   \
                                               ENETPHY_LINK_CAP_HD100 |  \
                                               ENETPHY_LINK_CAP_FD100 |  \
                                               ENETPHY_LINK_CAP_HD1000 | \
                                               ENETPHY_LINK_CAP_FD1000)

/*! @} */

/*! \brief Max extended configuration size, arbitrarily chosen. */
#define ENETPHY_EXTENDED_CFG_SIZE_MAX         (128U)

/*! \brief Enet PHY State Machine tick period. */
#define ENETPHY_FSM_TICK_PERIOD_MS            (100U)

/*! \brief Invalid PHY address indicator. */
#define ENETPHY_INVALID_PHYADDR               (~0U)

/*! \brief State timeout value to set to disable timeout */
#define ENETPHY_TIMEOUT_WAIT_FOREVER          (0xFFFFFFFFU)

/*! \brief State timeout value to set to expire immediately */
#define ENETPHY_TIMEOUT_NO_WAIT               (0U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief EnetPhy driver magic value, used to indicate if driver is open or not.
 */
typedef enum EnetPhy_Magic_e
{
    /*! Magic number used to identify when driver has been opened. */
    ENETPHY_MAGIC = 0xCADACADAU,

    /*! Magic number used to identify when driver is closed. */
    ENETPHY_NO_MAGIC = 0x0U,
} EnetPhy_Magic;

/*!
 * \brief MAC Media-Independent Interface (MII).
 */
typedef enum EnetPhy_Mii_e
{
    /*! \brief MII interface */
    ENETPHY_MAC_MII_MII = 0U,

    /*! \brief RMII interface */
    ENETPHY_MAC_MII_RMII,

    /*! \brief GMII interface */
    ENETPHY_MAC_MII_GMII,

    /*! \brief RGMII interface */
    ENETPHY_MAC_MII_RGMII,

    /*! \brief SGMII interface */
    ENETPHY_MAC_MII_SGMII,

    /*! \brief QSGMII interface */
    ENETPHY_MAC_MII_QSGMII,
} EnetPhy_Mii;

/*!
 * \brief MAC interface speed.
 */
typedef enum EnetPhy_Speed_e
{
    /*! 10 Mbps */
    ENETPHY_SPEED_10MBIT = 0U,

    /*! 100 Mbps */
    ENETPHY_SPEED_100MBIT,

    /*! 1 Gbps */
    ENETPHY_SPEED_1GBIT,

    /*! Speed determined automatically */
    ENETPHY_SPEED_AUTO,
} EnetPhy_Speed;

/*!
 * \brief MAC interface duplexity.
 */
typedef enum EnetPhy_Duplexity_e
{
    /*! Half duplex */
    ENETPHY_DUPLEX_HALF = 0U,

    /*! Full duplex */
    ENETPHY_DUPLEX_FULL,

    /*! Duplexity determined automatically */
    ENETPHY_DUPLEX_AUTO,
} EnetPhy_Duplexity;

/*!
 * \brief PHY version (ID).
 */
typedef struct EnetPhy_Version_s
{
    /*! Organizationally Unique Identifier (OUI) */
    uint32_t oui;

    /*! Manufacturer's model number */
    uint32_t model;

    /*! Revision number */
    uint32_t revision;
} EnetPhy_Version;

/*!
 * \brief PHY link status.
 */
typedef enum EnetPhy_LinkStatus_e
{
    /*! PHY got link up */
    ENETPHY_GOT_LINK = 0U,

    /*! PHY link is still up */
    ENETPHY_LINK_UP,

    /*! PHY lost link */
    ENETPHY_LOST_LINK,

    /*! PHY link is still down */
    ENETPHY_LINK_DOWN,
} EnetPhy_LinkStatus;

/*!
 * \brief Link speed and duplexity configuration.
 */
typedef struct EnetPhy_LinkCfg_s
{
    /*! Link speed */
    EnetPhy_Speed speed;

    /*! Duplexity */
    EnetPhy_Duplexity duplexity;
} EnetPhy_LinkCfg;

/*!
 * \brief PHY State-Machine time-out values.
 */
typedef struct EnetPhy_FsmTimeoutCfg_s
{
    /*! \brief FINDING state timeout (in ticks).
     *
     * Set to \ref ENETPHY_TIMEOUT_WAIT_FOREVER, to disable timeout.\n
     * Set to \ref ENETPHY_TIMEOUT_NO_WAIT, to expire immediately */
    uint32_t findingStateTicks;

    /*! \brief RESET_WAIT state timeout (in ticks).
     *
     * Set to \ref ENETPHY_TIMEOUT_WAIT_FOREVER, to disable timeout.\n
     * Set to \ref ENETPHY_TIMEOUT_NO_WAIT, to expire immediately */
    uint32_t resetWaitStateTicks;

    /*! \brief RESET_WAIT state residence time (in ticks). */
    uint32_t resetWaitStateResidenceTicks;

    /*! \brief NWAY_START state timeout (in ticks).
     *
     * Set to \ref ENETPHY_TIMEOUT_WAIT_FOREVER, to disable timeout.\n
     * Set to \ref ENETPHY_TIMEOUT_NO_WAIT, to expire immediately */
    uint32_t nwayStartStateTicks;

    /*! \brief NWAY_WAIT state timeout (in ticks).
     *
     * Set to \ref ENETPHY_TIMEOUT_WAIT_FOREVER, to disable timeout.\n
     * Set to \ref ENETPHY_TIMEOUT_NO_WAIT, to expire immediately */
    uint32_t nwayWaitStateTicks;

    /*! \brief LINK_WAIT state timeout (in ticks).
     *
     * Set to \ref ENETPHY_TIMEOUT_WAIT_FOREVER, to disable timeout.\n
     * Set to \ref ENETPHY_TIMEOUT_NO_WAIT, to expire immediately */
    uint32_t linkWaitStateTicks;

    /*! \brief Timeout if MDIX is enabled (in ticks). */
    uint32_t mdixTicks;
} EnetPhy_FsmTimeoutCfg;

/*!
 * \brief PHY configuration parameters.
 */
typedef struct EnetPhy_Cfg_s
{
    /*! MDIO group */
    uint32_t phyGroup;

    /*! PHY device address */
    uint32_t phyAddr;

    /*! Auto-negotiation advertise capabilities */
    uint32_t nwayCaps;

    /*! MDIX enable */
    bool mdixEn;

    /*! Whether PHY is strapped or not.  If strapping is present, no explicit
     *  PHY register configurations will take place and will only wait for
     *  link to be established */
    bool isStrapped;

    /*! Whether Isolate state is requested from the application */
    bool isIsolateStateReq;

    /*! Enable loopback once PHY is found */
    bool loopbackEn;

    /*! Enable master mode */
    bool masterMode;

    /*! Enable external clock source */
    bool extClkSource;

    /*! Skip PHY-specific extended configuration */
    bool skipExtendedCfg;

    /*! PHY state-machine timeout configuration */
    EnetPhy_FsmTimeoutCfg timeoutCfg;

    /*! Extended PHY-specific configuration */
    uint8_t extendedCfg[ENETPHY_EXTENDED_CFG_SIZE_MAX];

    /*! Size of the extended configuration */
    uint32_t extendedCfgSize;

} EnetPhy_Cfg;

/*!
 * \brief MDIO driver.
 */
typedef struct EnetPhy_Mdio_s
{
    /*!
     * \brief Check if PHY is alive.
     *
     * Checks if PHY is alive, either using an explicit register read or any other
     * mechanism supported by the MDIO peripheral (i.e. background BMSR reads).
     *
     * \param phyAddr    PHY device address
     * \param isAlive    Whether PHY is alive or not
     * \param args       Caller's arguments passed to the PHY driver at open time
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*isAlive)(uint32_t phyAddr,
                       bool *isAlive,
                       void *arg);

    /*!
     * \brief Check if PHY is linked.
     *
     * Checks if PHY is linked, either using an explicit register read or any other
     * mechanism supported by the MDIO peripheral.
     *
     * \param phyAddr    PHY device address
     * \param isLinked   Whether PHY is linked or not
     * \param args       Caller's arguments passed to the PHY driver at open time
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*isLinked)(uint32_t phyAddr,
                        bool *isLinked,
                        void *arg);

    /*!
     * \brief Read PHY register using Clause-22 frame.
     *
     * Reads a PHY register using a Clause-22 frame.
     *
     * \param group      User group (use 0 if single group is supported)
     * \param phyAddr    PHY device address
     * \param reg        Register address
     * \param val        Value read from register
     * \param args       Caller's arguments passed to the PHY driver at open time
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*readC22)(uint32_t group,
                       uint32_t phyAddr,
                       uint32_t reg,
                       uint16_t *val,
                       void *arg);

    /*!
     * \brief Write PHY register using Clause-22 frame.
     *
     * Writes a PHY register using a Clause-22 frame.
     *
     * \param group      User group (use 0 if single group is supported)
     * \param phyAddr    PHY device address
     * \param reg        Register address
     * \param val        Value to be written
     * \param args       Caller's arguments passed to the PHY driver at open time
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*writeC22)(uint32_t group,
                        uint32_t phyAddr,
                        uint32_t reg,
                        uint16_t val,
                        void *arg);

    /*!
     * \brief Read PHY register using Clause-45 frame.
     *
     * Reads a PHY register using a Clause-45 frame.  Returns #ENETPHY_ENOTSUPPORTED
     * if MDIO doesn't support Clause-45 frames.
     *
     * \param group      User group (use 0 if single group is supported)
     * \param phyAddr    PHY device address
     * \param reg        Register address
     * \param val        Value read from register
     * \param args       Caller's arguments passed to the PHY driver at open time
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*readC45)(uint32_t group,
                       uint32_t phyAddr,
                       uint8_t mmd,
                       uint16_t reg,
                       uint16_t *val,
                       void *arg);

    /*!
     * \brief Write PHY register using Clause-45 frame.
     *
     * Writes a PHY register using a Clause-45 frame. Returns #ENETPHY_ENOTSUPPORTED
     * if MDIO doesn't support Clause-45 frames.
     *
     * \param group      User group (use 0 if single group is supported)
     * \param phyAddr    PHY device address
     * \param reg        Register address
     * \param val        Value to be written
     * \param args       Caller's arguments passed to the PHY driver at open time
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*writeC45)(uint32_t group,
                        uint32_t phyAddr,
                        uint8_t mmd,
                        uint16_t reg,
                        uint16_t val,
                        void *arg);
} EnetPhy_Mdio;

/*!
 * \brief MDIO driver handle.
 */
typedef EnetPhy_Mdio *EnetPhy_MdioHandle;

/*!
 * \brief PHY specific driver handle.
 */
typedef struct EnetPhy_Drv_s *EnetPhyDrv_Handle;

/*!
 * \brief PHY driver state-machine states.
 */
typedef enum EnetPhy_FsmState_e
{
    /*! \brief INIT state */
    ENETPHY_FSM_STATE_INIT = 0U,

    /*! \brief FINDING state */
    ENETPHY_FSM_STATE_FINDING,

    /*! \brief RESET_WAIT state */
    ENETPHY_FSM_STATE_RESET_WAIT,

    /*! \brief ENABLE state */
    ENETPHY_FSM_STATE_ENABLE,

    /*! \brief FOUND state */
    ENETPHY_FSM_STATE_FOUND,

    /*! \brief NWAY_START state (auto-negotiation path) */
    ENETPHY_FSM_STATE_NWAY_START,

    /*! \brief NWAY_WAIT state (auto-negotiation path) */
    ENETPHY_FSM_STATE_NWAY_WAIT,

    /*! \brief LINK_WAIT state */
    ENETPHY_FSM_STATE_LINK_WAIT,

    /*! \brief LINKED state */
    ENETPHY_FSM_STATE_LINKED,

    /*! \brief LOOPBACK state */
    ENETPHY_FSM_STATE_LOOPBACK,

    /*! \brief ISOLATE state*/
    ENETPHY_FSM_STATE_ISOLATE
} EnetPhy_FsmState;

/*!
 * \brief PHY driver FSM state.
 */
typedef struct EnetPhy_State_s
{
    /*! PHY state-machine state */
    EnetPhy_FsmState fsmState;

    /*! Whether the PHY state-machine state has changed */
    bool fsmStateChanged;

    /*! PHY speed (auto-negotiated or manually set) */
    EnetPhy_Speed speed;

    /*! PHY duplexity (auto-negotiated or manually set) */
    EnetPhy_Duplexity duplexity;

    /*! Timeout (ticks) */
    uint32_t timeout;

    /*! Residence time (ticks) */
    uint32_t residenceTime;

    /*! Whether PHY is auto-negotiation capable */
    bool isNwayCapable;

    /*! Whether auto-negotiation is to be used or not */
    bool enableNway;

    /*! Whether manual mode needs to be configured */
    bool needsManualCfg;

    /*! Whether auto-negotiation advertisement needs to be configured */
    bool needsNwayCfg;

    /*! Refined link capability mask (app, SoC, PHY) */
    uint32_t linkCaps;

    /*! PHY link capability mask */
    uint32_t phyLinkCaps;

    /*! Whether PHY loopback is enabled or not */
    bool loopbackEn;

    /*! Whether MDIX switch is needed or not */
    bool needsMdixSwitch;

    /*! Whether MDIX is enabled or not (MDI) */
    bool enableMdix;
} EnetPhy_State;

/*!
 * \brief PHY driver object.
 */
typedef struct EnetPhy_Obj_s
{
    /*! MDIO handle used to access PHY registers */
    EnetPhy_MdioHandle hMdio;

    /*! PHY configuration params */
    EnetPhy_Cfg phyCfg;

    /*! MII interface type */
    EnetPhy_Mii mii;

    /*! MAC port supported capabilities */
    uint32_t macCaps;

    /*! Port Link configuration (speed, duplexity) */
    EnetPhy_LinkCfg linkCfg;

    /*! State-machine timeout configuration */
    EnetPhy_FsmTimeoutCfg timeoutCfg;

    /*! State-machine state */
    EnetPhy_State state;

    /*! PHY group */
    uint32_t group;

    /*! PHY device address */
    uint32_t addr;

    /*! Requested link capability mask */
    uint32_t reqLinkCaps;

    /*! PHY driver */
    EnetPhyDrv_Handle hDrv;

    /*! Magic number indicating that this object is in use */
    EnetPhy_Magic magic;

    /*! Caller-provided arguments to be used in MDIO driver calls */
    void *mdioArgs;
} EnetPhy_Obj;

/*!
 * \brief PHY driver object handle.
 *
 * PHY driver opaque handle used to call any PHY related APIs.
 */
typedef struct EnetPhy_Obj_s *EnetPhy_Handle;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize PHY config params.
 *
 * Initializes PHY driver configuration parameters.
 *
 * \param phyCfg   PHY configuration params
 */
void EnetPhy_initCfg(EnetPhy_Cfg *phyCfg);

/*!
 * \brief Set PHY extended parameters.
 *
 * Sets the PHY-specific extended parameters to the PHY config structure.
 *
 * \param phyCfg            Pointer to the PHY config
 * \param extendedCfg       Pointer to the PHY extended config
 * \param extendedCfgSize   Size of the PHY extended config
 */
void EnetPhy_setExtendedCfg(EnetPhy_Cfg *phyCfg,
                            const void *extendedCfg,
                            uint32_t extendedCfgSize);

/*!
 * \brief Open the PHY driver.
 *
 * Open the Ethernet PHY driver for the given MAC port number. The PHY driver
 * takes PHY specific configuration parameters, the MAC port type connection
 * and the desired link configuration (auto or manual).
 *
 * \param phyCfg      PHY configuration params
 * \param mii         PHY MII interface type
 * \param linkCfg     Link configuration (speed and duplexity)
 * \param macPortCaps MAC port speed/duplex capabilities. It's a bit mask of
 *                    \ref EnetPhy_LinkCaps
 * \param hMdio       MDIO driver to be used for PHY register read/write
 * \param mdioArgs    Private data passed to the MDIO driver functions
 *
 * \return PHY device handle if successful, NULL otherwise.
 */
EnetPhy_Handle EnetPhy_open(const EnetPhy_Cfg *phyCfg,
                            EnetPhy_Mii mii,
                            const EnetPhy_LinkCfg *linkCfg,
                            uint32_t macPortCaps,
                            EnetPhy_MdioHandle hMdio,
                            void *mdioArgs);

/*!
 * \brief Close the PHY driver.
 *
 * Closes the Ethernet PHY driver.
 *
 * \param hPhy     PHY device handle
 */
void EnetPhy_close(EnetPhy_Handle hPhy);

/*!
 * \brief Run PHY state machine.
 *
 * Runs the PHY FSM.
 *
 * \param hPhy     PHY device handle
 *
 * \return Whether PHY got or lost link, or no change.
 */
EnetPhy_LinkStatus EnetPhy_tick(EnetPhy_Handle hPhy);

/*!
 * \brief Get PHY id.
 *
 * Gets the device ID of a PHY, read from IDR1 and IDR2 registers.
 *
 * \param hPhy     PHY device handle
 * \param version  Pointer to PHY version.
 *
 * \return \ref EnetPhy_ErrorCodes
 */
int32_t EnetPhy_getId(EnetPhy_Handle hPhy,
                      EnetPhy_Version *version);

/*!
 * \brief Get PHY alive status.
 *
 * Gets the PHY alive status. Whether PHY is responding to read accesses.
 *
 * \param hPhy     PHY device handle
 *
 * \return true if PHY is alive, false otherwise
 */
bool EnetPhy_isAlive(EnetPhy_Handle hPhy);

/*!
 * \brief Get link status.
 *
 * Gets the link status: linked or not, based on driver's state machine.
 * The PHY driver state machine can take a little longer to detect link up
 * because it runs on tick period intervals and need to traverse few states
 * to reach link up FSM state.
 *
 * \param hPhy     PHY device handle
 *
 * \return true if PHY is linked, false otherwise
 */
bool EnetPhy_isLinked(EnetPhy_Handle hPhy);

/*!
 * \brief Get link configuration.
 *
 * Gets the link configuration, that is, the configuration that the PHY has
 * negotiated with the link partner or the manual link configuration it was
 * set to.
 *
 * \param hPhy     PHY device handle
 * \param linkCfg  Link configuration
 *
 * \return \ref EnetPhy_ErrorCodes
 */
int32_t EnetPhy_getLinkCfg(EnetPhy_Handle hPhy,
                           EnetPhy_LinkCfg *linkCfg);

/*!
 * \brief Read PHY register.
 *
 * Reads a PHY register. It's not meant for extended registers.
 *
 * \param hPhy     PHY device handle
 * \param reg      Register number
 * \param val      Pointer to the read value
 *
 * \return \ref EnetPhy_ErrorCodes
 */
int32_t EnetPhy_readReg(EnetPhy_Handle hPhy,
                        uint32_t reg,
                        uint16_t *val);

/*!
 * \brief Write PHY register.
 *
 * Writes a PHY register. It's not meant for extended registers.
 *
 * \param hPhy     PHY device handle
 * \param reg      Register number
 * \param val      Value to be written
 *
 * \return \ref EnetPhy_ErrorCodes
 */
int32_t EnetPhy_writeReg(EnetPhy_Handle hPhy,
                         uint32_t reg,
                         uint16_t val);

/*!
 * \brief Read-modify-write PHY register.
 *
 * Read-modify-write a PHY register. It's not meant for extended registers.
 *
 * \param hPhy     PHY device handle
 * \param reg      Register number
 * \param mask     Bitmask to be applied on read value and value to be written
 * \param val      Value to be written
 *
 * \return \ref EnetPhy_ErrorCodes
 */
int32_t EnetPhy_rmwReg(EnetPhy_Handle hPhy,
                       uint32_t reg,
                       uint16_t mask,
                       uint16_t val);

/*!
 * \brief Read PHY extended register.
 *
 * Reads a PHY extended register.
 *
 * \param hPhy     PHY device handle
 * \param reg      Register number
 * \param val      Pointer to the read value
 *
 * \return \ref EnetPhy_ErrorCodes
 */
int32_t EnetPhy_readExtReg(EnetPhy_Handle hPhy,
                           uint32_t reg,
                           uint16_t *val);

/*!
 * \brief Write PHY extended register.
 *
 * Writes a PHY extended register.
 *
 * \param hPhy     PHY device handle
 * \param reg      Register number
 * \param val      Value to be written
 *
 * \return \ref EnetPhy_ErrorCodes
 */
int32_t EnetPhy_writeExtReg(EnetPhy_Handle hPhy,
                            uint32_t reg,
                            uint16_t val);

/*!
 * \brief Read-modify-write PHY extended register.
 *
 * Read-modify-write a PHY extended register.
 *
 * \param hPhy     PHY device handle
 * \param reg      Register number
 * \param mask     Bitmask to be applied on read value and value to be written
 * \param val      Value to be written
 *
 * \return \ref EnetPhy_ErrorCodes
 */
int32_t EnetPhy_rmwExtReg(EnetPhy_Handle hPhy,
                          uint32_t reg,
                          uint16_t mask,
                          uint16_t val);

/*!
 * \brief Read PHY register using Clause-45 frame.
 *
 * Reads a PHY register using Clause-45 frame.
 *
 * \param hPhy     PHY device handle
 * \param mmd      MMD
 * \param reg      Register number
 * \param val      Pointer to the read value
 *
 * \return \ref EnetPhy_ErrorCodes
 */
int32_t EnetPhy_readC45Reg(EnetPhy_Handle hPhy,
                           uint8_t mmd,
                           uint32_t reg,
                           uint16_t *val);

/*!
 * \brief Write PHY register using Clause-45 frame.
 *
 * Writes a PHY register using Clause-45 frame.
 *
 * \param hPhy     PHY device handle
 * \param mmd      MMD
 * \param reg      Register number
 * \param val      Value to be written
 *
 * \return \ref EnetPhy_ErrorCodes
 */
int32_t EnetPhy_writeC45Reg(EnetPhy_Handle hPhy,
                            uint8_t mmd,
                            uint32_t reg,
                            uint16_t val);

/*!
 * \brief Read-modify-write PHY register using Clause-45 frame.
 *
 * Read-modify-write a PHY register using Clause-45 frame.
 *
 * \param hPhy     PHY device handle
 * \param mmd      MMD
 * \param reg      Register number
 * \param mask     Bitmask to be applied on read value and value to be written
 * \param val      Value to be written
 *
 * \return \ref EnetPhy_ErrorCodes
 */
int32_t EnetPhy_rmwC45Reg(EnetPhy_Handle hPhy,
                          uint8_t mmd,
                          uint32_t reg,
                          uint16_t mask,
                          uint16_t val);

/*!
 * \brief Print all PHY registers.
 *
 * Prints all registers of a PHY.
 *
 * \param hPhy     PHY device handle
 */
void EnetPhy_printRegs(EnetPhy_Handle hPhy);

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

#endif /* ENETPHY_H_ */

/*! @} */
