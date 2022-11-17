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
 * \file  enet_phymdio_dflt.h
 *
 * \brief This file contains the interface to the default implementation of
 *        the MDIO driver defined by the Ethernet PHY driver (EnetPhy).
 */

#ifndef ENET_PHYMDIO_DFLT_H_
#define ENET_PHYMDIO_DFLT_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <include/phy/enetphy.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Get handle to the default Enet MDIO implementation.
 *
 * Gets a handle to the default EnetPhy MDIO driver implementation which is
 * based on the Enet MDIO module.
 *
 * \return Handle to the Enet based MDIO default implementation.
  */
EnetPhy_MdioHandle EnetPhyMdioDflt_getPhyMdio(void);

/*!
 * \brief Run an IOCTL operation on a PHY.
 *
 * Runs an IOCTL operation on a given PHY using the ENETPHY driver APIs.
 * This IOCTL function can be used by Enet Peripheral drivers to handle
 * PHY IOCTL commands if they use an ENETPHY driver.
 *
 * \param hPhy         PHY handle
 * \param cmd          IOCTL command Id
 * \param prms         IOCTL parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetPhyMdioDflt_ioctl(EnetPhy_Handle hPhy,
                              uint32_t cmd,
                              Enet_IoctlPrms *prms);

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

#endif /* ENET_PHYMDIO_DFLT_H_ */
