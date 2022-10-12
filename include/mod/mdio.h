/*
 *  Copyright (c) Texas Instruments Incorporated 2022
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
 * \file  include/mod/mdio.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        MDIO module interface.
 */

/*!
 * \ingroup  ENET_MOD_MDIO
 * \defgroup MDIO_MOD CPSW Management Data Input/Output (MDIO)
 *
 * The MDIO module implements the MDIO-generic IOCTL commands defined in
 * ENET_MOD_MDIO API set.
 *
 * MDIO clocks:
 * - CPSW_CPPI_CLK - CPSW main clock.
 *
 * Interrupts:
 * - MDIO_PEND - MDIO interrupt. Peripheral driver must call
 *   MDIO_IOCTL_HANDLE_INTR command to handle the MDIO interrupt.
 *
 * Features:
 * - MDIO_FEATURE_CLAUSE45 - Clause-45 frame feature.
 *
 * Compile-time configuration:
 * - #ENET_CFG_MDIO_CLAUSE45 - Clause-45 frame support.
 *
 * @{
 */

#ifndef MDIO_H_
#define MDIO_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <include/core/enet_mod_mdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Maximum number of PHYs supported on MDIO bus. */
#define MDIO_MAX_PHY_CNT                      (31U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief MDIO operating mode.
 */
typedef enum Mdio_OpMode_e
{
    /*! Normal mode */
    MDIO_MODE_NORMAL = 0U,

    /*! State change monitor mode */
    MDIO_MODE_STATE_CHANGE_MON,

    /*! Manual mode (slow GPIO, not used for MDIO PHY operations. Used only during MDIO pins software bit banging) */
    MDIO_MODE_MANUAL,
} Mdio_OpMode;

/*!
 * \brief MDIO module configuration.
 *
 * Configuration parameters for the MDIO module.
 */
typedef struct Mdio_Cfg_s
{
    /*! MDIO module operating mode */
    Mdio_OpMode mode;

    /*! MDIO bus clock (MDCLK) frequency in Hz */
    uint32_t mdioBusFreqHz;

    /*! Polling inter packet gap frequency in Hz */
    uint32_t phyStatePollFreqHz;

    /*! Indicates which PHY addresses have polling enabled.
     *  The ENET_MDIO_PHY_ADDR_MASK macro should be used to create the mask.
     *  Only two PHYs can be monitored in Normal Mode, so only two bits must
     *  be set if operating in that mode. */
    uint32_t pollEnMask;

    /*! Indicates which PHY addresses will use Clause-45 frame format.
     *  The ENET_MDIO_PHY_ADDR_MASK macro should be used to create the mask */
    uint32_t c45EnMask;

    /*! MDIO module can be shared by multiple peripherals, so only one peripheral
     *  (master) must perform the MDIO initial configuration and be skipped by
     *  the other peripherals (slaves).
     *  There must be only one peripheral marked as master among the those that
     *  share the MDIO.  This condition is not enforced in driver, so it's
     *  application's responsibility this requirement is met. */
    bool isMaster;

    /*! Config to disable MDIO state machine on MDIO_open .
     *  By default MDIO state machine is enabled on MDIO_open
     *  Setting this flag, will cause MDIO to not enable MDIO state
     *  machine on open. The MDIO state machine can be enabled at 
     *  a later point of time via IOCTL. This option allows
     *  Synchronizing externally managed PHY with the MDIO 
     *  whereby external PHY initialization is done and then
     *  MDIO state machine can be enabled so that any linkup
     *  interrupts are not missed
     */
    bool disableStateMachineOnInit;

} Mdio_Cfg;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize MDIO configuration parameters.
 *
 * \param mdioCfg   Configuration parameters to be initialized
 */
void Mdio_initCfg(Mdio_Cfg *mdioCfg);

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

#endif /* MDIO_H_ */

/*! @} */
