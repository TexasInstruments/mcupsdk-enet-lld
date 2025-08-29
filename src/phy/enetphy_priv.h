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
 * \file  enetphy_priv.h
 *
 * \brief This file contains internal type definitions and helper macros for the
 *        Ethernet PHY interface.
 */

#ifndef ENETPHY_PRIV_H_
#define ENETPHY_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/phy/enetphy.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Macro to perform round-up division. */
#define ENETPHY_DIV_ROUNDUP(val, div)         (((val) + (div) - 1) / (div))

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief PHY specific driver.
 *
 * PHY specific driver implementation. This driver code must not modify registers
 * and/or bitfields that are being modified by the main PHY driver. Otherwise,
 * PHY driver state machine may not work as the underlying state of the PHY would
 * be inconsistent.
 */
typedef struct EnetPhy_Drv_s
{
    /*!
     * \brief Driver name.
     *
     * Name of the PHY-specific driver.
     */
    const char *name;

    /*!
     * \brief Check if driver supports a PHY model identified by its version.
     *
     * PHY-specific function that drivers must implement for upper check if the
     * PHY driver supports a PHY model identified by its version from ID1 and
     * ID2 registers.
     *
     * Note that a given PHY driver can support multiple PHY models.
     *
     * \param hPhy     PHY device handle
     * \param version  PHY version from ID registers
     *
     * \return Whether PHY model is supported or not
     */

    bool (*isPhyDevSupported)(void *pArgs,
                            const void *pVersion,
                            const void *pCfg);

    /*!
     * \brief Check if driver supports a MII interface type.
     *
     * PHY-specific function that drivers must implement for upper layer to check
     * whether a MAC mode is supported by the PHY driver or not.
     *
     * \param hPhy     PHY device handle
     * \param mii      MII interface
     *
     * \return Whether MII interface type is supported or not
     */
    bool (*isMacModeSupported)(void *pArgs,
                               void *pMii);

    /*!
     * \brief PHY specific configuration.
     *
     * PHY-specific function that drivers must implement to configure the PHY
     * device.  The configuration can be composed of generic and PHY-specific
     * parameter (via extended config).
     *
     * \param hPhy     PHY device handle
     * \param cfg      PHY configuration parameter
     * \param mii      MII interface
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*config)(void *pArgs,
                      const void *pCfg,
                      void *pMii);

    /*!
     * \brief PHY specific soft reset.
     *
     * PHY-specific function that drivers must implement to start a soft-reset
     * operation.
     *
     * \param hPhy     PHY device handle
     */
    void (*reset)(void *pArgs);

    /*!
     * \brief PHY specific soft reset status.
     *
     * PHY-specific function that drivers must implement to check if soft-reset
     * operation is complete.
     *
     * \param hPhy     PHY device handle
     *
     * \return Whether soft-reset is complete or not.
     */
    bool (*isResetComplete)(void *pArgs);

    /*!
     * \brief Read PHY extended register.
     *
     * PHY-specific function that drivers must implement to read extended
     * registers.
     *
     * \param hPhy     PHY device handle
     * \param reg      Register number
     * \param val      Pointer to the read value
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*readExtReg)(void *pArgs,
                          uint32_t reg,
                          uint16_t *val);

    /*!
     * \brief Write PHY register.
     *
     * PHY-specific function that drivers must implement to write extended
     * registers.
     *
     * \param hPhy     PHY device handle
     * \param reg      Register number
     * \param val      Value to be written
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*writeExtReg)(void *pArgs,
                           uint32_t reg,
                           uint16_t val);

    /*!
     * \brief Read-modify-write PHY extended register.
     *
     * PHY-specific function that drivers must implement to read-write-modify
     * extended registers.
     *
     * \param group      User group (use 0 if single group is supported)
     * \param phyAddr    PHY device address
     * \param reg        Register address
     * \param val        Value read from register
     */
    int32_t (*rmwExtReg)(void *pArgs,
                         uint32_t group,
                         uint32_t reg,
                         uint16_t mask,
                         uint16_t val);

    /*! Print PHY registers */
    void (*printRegs)(void *pArgs, uint32_t addr);

    /*!
     * \brief Adjust PHY PTP clock frequency.
     *
     * Optional PHY function to adjust PTP clock frequency.
     * This function can only be supported when the PHY has a built-in PTP clock.
     *
     * \param hPhy     PHY device handle
     * \param ppb      Part per billion
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*adjPtpFreq)(EnetPhy_Handle hPhy,
                          int64_t ppb);

    /*!
     * \brief Adjust PHY PTP clock phase.
     *
     * Optional PHY function to adjust PTP clock phase.
     * This function can only be supported when the PHY has a built-in PTP clock.
     *
     * \param hPhy     PHY device handle
     * \param offset   Offset to current clock time in nanosec unit.
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*adjPtpPhase)(EnetPhy_Handle hPhy,
                          int64_t offset);

    /*!
     * \brief Get current PHY PTP clock time.
     *
     * Optional PHY function to get current PHY PTP clock time.
     * This function can only be supported when the PHY has a built-in PTP clock.
     *
     * \param hPhy     PHY device handle
     * \param ts64     Output current PTP clock time in nanosec unit.
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*getPtpTime)(EnetPhy_Handle hPhy,
                          uint64_t *ts64);

    /*!
     * \brief Set PHY PTP clock time.
     *
     * Optional PHY function to set PHY PTP clock time.
     * This function can only be supported when the PHY has a built-in PTP clock.
     *
     * \param hPhy     PHY device handle
     * \param ts64     PTP time in nanosec unit will be set.
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*setPtpTime)(EnetPhy_Handle hPhy,
                          uint64_t ts64);

    /*!
     * \brief Get PHY PTP TX packet timestamp.
     *
     * Optional PHY function to get PHY PTP TX packet timestamp.
     * This function can only be supported when the PHY has a built-in PTP clock.
     *
     * \param hPhy     PHY device handle
     * \param domain   PTP domain (in the packet header)
     * \param msgType  PTP message type (in the packet header)
     * \param seqId    PTP packet sequence ID (in the packet header)
     * \param ts64     Output PTP TX packet timestamp in nanosec unit.
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*getPtpTxTime)(EnetPhy_Handle hPhy,
                            uint32_t domain,
                            uint32_t msgType,
                            uint32_t seqId,
                            uint64_t *ts64);

    /*!
     * \brief Get PHY PTP RX packet timestamp.
     *
     * Optional PHY function to get PHY PTP RX packet timestamp.
     * This function can only be supported when the PHY has a built-in PTP clock.
     *
     * \param hPhy     PHY device handle
     * \param domain   PTP domain (in the packet header)
     * \param msgType  PTP message type (in the packet header)
     * \param seqId    PTP packet sequence ID (in the packet header)
     * \param ts64     Output PTP RX packet timestamp in nanosec unit.
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*getPtpRxTime)(EnetPhy_Handle hPhy,
                            uint32_t domain,
                            uint32_t msgType,
                            uint32_t seqId,
                            uint64_t *ts64);

    /*!
     * \brief Add PHY PTP TX packet info to a waiting TX timestamp list.
     *
     * Optional PHY function to get PHY PTP TX packet timestamp.
     * This function can only be supported when the PHY has a built-in PTP clock.
     *
     * \param hPhy     PHY device handle
     * \param domain   PTP domain (in the packet header)
     * \param msgType  PTP message type (in the packet header)
     * \param seqId    PTP packet sequence ID (in the packet header)
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*waitPtpTxTime)(EnetPhy_Handle hPhy,
                             uint32_t domain,
                             uint32_t msgType,
                             uint32_t seqId);

    /*!
     * \brief Process PHY status frame.
     *
     * Optional PHY function to process PHY status frame.
     * This function can only be supported when the PHY has a built-in PTP clock.
     *
     * \param hPhy     PHY device handle
     * \param frame    Ethernet PHY status frame
     * \param size     Frame size
     * \param types    Types of processed frame
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*procStatusFrame)(EnetPhy_Handle hPhy,
                               uint8_t *frame,
                               uint32_t size,
                               uint32_t *types);

    /*!
     * \brief Get PHY status frame header.
     *
     * Optional PHY function to process PHY status frame.
     * This function can only be supported when the PHY has a built-in PTP clock.
     *
     * \param hPhy     PHY device handle
     * \param ethhdr   Buffer to get the ethernet header of the PHY status frame.
     * \param size     Buffer size (at least 14 bytes)
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*getStatusFrameEthHeader)(EnetPhy_Handle hPhy,
                                       uint8_t *ethhdr,
                                       uint32_t size);

    /*!
     * \brief Enable or disable the PHY PTP module
     *
     * Optional PHY function to enable or disable the PHY PTP module.
     * This function can only be supported when the PHY has a built-in PTP clock.
     *
     * \param hPhy     PHY device handle
     * \param on       Flag indicate enable (on=true) or disable(on=false) PTP module
     * \param srcMacStatusFrameType      The PHY-specific src MAC of the status frame.
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*enablePtp)(EnetPhy_Handle hPhy,
                         bool on,
                         uint32_t srcMacStatusFrameType);

    /*!
     * \brief Provide timer tick to the driver.
     *
     * Provide timer tick to the driver.
     *
     * \param hPhy     PHY device handle
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*tickDriver)(EnetPhy_Handle hPhy);

    /*!
     * \brief Enable/Disable an event capture on a PHY GPIO pin.
     *
     * Optional PHY function to enable/disable an event capture on a PHY GPIO pin.
     * This function can only be supported when the PHY has a built-in PTP clock
     * that support event capture.
     *
     * \param hPhy     PHY device handle
     * \param eventIdx Event index
     * \param falling  Capture event on falling edge or rising edge if falling is false.
     * \param on       Enable when on is true, otherwise disable the event.
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*enableEventCapture)(EnetPhy_Handle hPhy, uint32_t eventIdx,
                bool falling, bool on);

    /*!
     * \brief Enable/Disable trigger output on a GPIO pin.
     *
     * Optional PHY function to enable/disable trigger output on a GPIO pin.
     * This function can only be supported when the PHY has a built-in PTP clock
     * that support trigger output.
     *
     * \param hPhy       PHY device handle
     * \param triggerIdx Trigger index
     * \param start      Start trigger time in nanosec unit.
     * \param period     Period of the pulse in nanosec unit.
     *                   Disable the trigger if the period is equal to 0.
     * \param repeat     Repeated pulse or one shot pulse if repeat is false.
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*enableTriggerOutput)(EnetPhy_Handle hPhy, uint32_t triggerIdx,
                uint64_t start, uint64_t period, bool repeat);

    /*!
     * \brief Get event timestamp
     *
     * Get event timestamp
     *
     * \param hPhy       PHY device handle
     * \param eventIdx   Output event index
     * \param seqId      Output event sequence identifier
     * \param ts64       Output event timestamp
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*getEventTs)(EnetPhy_Handle hPhy, uint32_t *eventIdx,
                uint32_t *seqId, uint64_t *ts64);


    /*!
     * \brief Provides selected speed and duplex when link is up
     *
     * Accessing vendor specific or extended registers based on type of PHY to provide
     * agreed speed and duplex mode for actual connection.
     *
     * \param hPhy           PHY device handle
     * \param pConfig        Contains speed/duplex combination used for actual
     *                       connection. See \ref Phy_Link_SpeedDuplex_e.
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*getSpeedDuplex)(EthPhyDrv_Handle hPhy, Phy_Link_SpeedDuplex* pConfig);

    /*!
     * \brief config Media Clock
     *
     * config Media Clock
     *
     * \param hPhy               PHY device handle
     * \param isMaster           Media Clock Master/Slave Mode
     * \param streamIDMatchValue Stream ID Match value for CRF Parsing
     * \param enTrigOut          If true, starts 100Hz phase aligned signal
     *                           to the media clock at LED0
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*configMediaClock)(EthPhyDrv_Handle hPhy, bool isMaster,
                 uint8_t *streamIDMatchValue, bool enTrigOut);
    /*!
     * \brief Nudge Codec Clock
     *
     * Nudge Codec Clock
     *
     * \param hPhy         PHY device handle
     * \param NudgeValue   int8_t value to nudge Codec clock.
     *
     * \return \ref EnetPhy_ErrorCodes
     */
    int32_t (*nudgeCodecClock)(EthPhyDrv_Handle hPhy, int8_t nudgeValue);
} EnetPhy_Drv;


typedef struct EnetPhy_DrvInfoTbl_s
{
    uint32_t numHandles;
    const EthPhyDrv_If *hPhyDrvList;
} EnetPhy_DrvInfoTbl;

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

#endif /* ENETPHY_PRIV_H_ */
