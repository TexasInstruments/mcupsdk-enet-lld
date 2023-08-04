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
 * \file  enet_mod.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Enet Module interface.
 */

#ifndef ENET_MOD_H_
#define ENET_MOD_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_types.h>
#include <include/core/enet_ioctl.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Convert module specific handle to EnetMod generic handle. */
#define ENET_MOD(mod)                         ((EnetMod_Handle)mod)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief Ethernet Module handle.
 *
 * Ethernet Module handle used to call any EnetMod related APIs.
 */
typedef struct EnetMod_Obj_s *EnetMod_Handle;

/*!
 * \brief Open and initialize the Enet Module.
 *
 * Opens and initializes the Enet Module with the configuration parameters
 * provided by the caller.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters
 * \param cfgSize   Size of the configuration parameters
 *
 * \return \ref Enet_ErrorCodes
 */
typedef int32_t (*const EnetMod_Open)(EnetMod_Handle hMod,
                                Enet_Type enetType,
                                uint32_t instId,
                                const void *cfg,
                                uint32_t cfgSize);

/*!
 * \brief Rejoin the Enet Module.
 *
 * Reopens the Enet Module, but doesn't perform any hardware initialization.
 * This function is expected to be called to attach to a running module.
 *
 * \param hMod      Enet Module opaque handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return \ref Enet_ErrorCodes
 */
typedef int32_t (* const EnetMod_Rejoin)(EnetMod_Handle hMod,
                                  Enet_Type enetType,
                                  uint32_t instId);

/*!
 * \brief Issue an operation on the Enet Module.
 *
 * Issues a control operation on the Enet Module.
 *
 * \param hMod         Enet Module opaque handle
 * \param cmd          IOCTL command Id
 * \param prms         IOCTL parameters
 *
 * \return \ref Enet_ErrorCodes
 */
typedef int32_t (* const EnetMod_Ioctl)(EnetMod_Handle hMod,
                                 uint32_t cmd,
                                 Enet_IoctlPrms *prms);

/*!
 * \brief Close the Enet Module.
 *
 * Closes the Enet Module.
 *
 * \param hMod        Enet Module opaque handle
 */
typedef void (* const EnetMod_Close)(EnetMod_Handle hMod);

/*!
 * \brief Saves and closes the Enet Module context before resetting
 *
 * Saves the Enet Module.
 *
 * \param hMod        Enet Module opaque handle
 */
typedef void (* const EnetMod_SaveCtxt)(EnetMod_Handle hMod);

/*!
 * \brief Restores and opens the Enet Module after reset
 *
 * Restores and opens the Enet Module with the configuration parameters
 * provided by the caller.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters
 * \param cfgSize   Size of the configuration parameters
 *
 * \return \ref Enet_ErrorCodes
 */
typedef int32_t (* const EnetMod_RestoreCtxt)(EnetMod_Handle hMod,
                                              Enet_Type enetType,
                                              uint32_t instId,
                                              const void *cfg,
                                              uint32_t cfgSize);
/*!
 * \brief Ethernet Module object.
 */
typedef struct EnetMod_Obj_s
{
    /*! Module name */
    const char *name;

    /*! Module's physical address */
    uint64_t physAddr;

    /*! Module's virtual address */
    void *virtAddr;

    /*! Module's second physical address, if needed */
    uint64_t physAddr2;

    /*! Module's second virtual address, if needed */
    void *virtAddr2;

    /*! Module features */
    uint32_t features;

    /*! Module's applicable errata */
    uint32_t errata;

    /*! Magic number indicating if the module has been opened */
    Enet_Magic magic;

    /*! Pointer to the EnetMod open function */
    EnetMod_Open open;

    /*! Pointer to the EnetMod rejoin function */
    EnetMod_Rejoin rejoin;

    /*! Pointer to the EnetMod ioctl function */
    EnetMod_Ioctl ioctl;

    /*! Pointer to the EnetMod close function */
    EnetMod_Close close;

    /*! Pointer to the EnetMod saveCtxt function */
    EnetMod_SaveCtxt saveCtxt;

    /*! Pointer to the EnetMod restoreCtxt function */
    EnetMod_RestoreCtxt restoreCtxt;
} EnetMod_Obj;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Wrapper to open and initialize an Enet Module.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters
 * \param cfgSize   Size of the configuration parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetMod_open(EnetMod_Handle hMod,
                     Enet_Type enetType,
                     uint32_t instId,
                     const void *cfg,
                     uint32_t cfgSize);

/*!
 * \brief Wrapper to rejoin an Enet Module.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetMod_rejoin(EnetMod_Handle hMod,
                       Enet_Type enetType,
                       uint32_t instId);

/*!
 * \brief Wrapper function to issue an operation on an Enet Module.
 *
 * \param hMod         Enet Module handle
 * \param cmd          IOCTL command Id
 * \param prms         IOCTL parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetMod_ioctl(EnetMod_Handle hMod,
                      uint32_t cmd,
                      Enet_IoctlPrms *prms);

/*!
 * \brief Wrapper function to register IOCTL for Macport Module.
          This function is defined for the cases when ONLY second
          macport module is opened without opening the first Macport
          module.
 *
 * \param hMod         Enet Module handle
 * \param cmdBase      IOCTL command Base
 * \param cmd          IOCTL command Id
 * \param prms         IOCTL parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetMod_registerMacportIoctlHandler(EnetMod_Handle hMod,
                                            uint32_t cmdBase,
                                            uint32_t cmd,
                                            Enet_IoctlPrms *prms);
/*!
 * \brief Wrapper function to issue an operation on an Enet Module
 *        from ISR context.
 *
 * \param hMod         Enet Module handle
 * \param cmd          IOCTL command Id
 * \param prms         IOCTL parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetMod_ioctlFromIsr(EnetMod_Handle hMod,
                             uint32_t cmd,
                             Enet_IoctlPrms *prms);

/*!
 * \brief Wrapper function to close an Enet Module.
 *
 * \param hMod         Enet Module handle
 */
void EnetMod_close(EnetMod_Handle hMod);

/*!
 * \brief Wrapper function to save and close Enet Module.
 *
 * \param hMod         Enet Module handle
 */
void EnetMod_saveCtxt(EnetMod_Handle hMod);

/*!
 * \brief Wrapper to retore and open Enet Module.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters
 * \param cfgSize   Size of the configuration parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetMod_restoreCtxt(EnetMod_Handle hMod,
                            Enet_Type enetType,
                            uint32_t instId,
                            const void *cfg,
                           uint32_t cfgSize);
/*!
 * \brief Check if Enet Module is open or not.
 *
 * \param hMod         Enet Module handle
 *
 * \return Module's open status.
 */
static inline bool EnetMod_isOpen(EnetMod_Handle hMod);

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline bool EnetMod_isOpen(EnetMod_Handle hMod)
{
    return (hMod->magic == ENET_MAGIC);
}

#ifdef __cplusplus
}
#endif

#endif /* ENET_MOD_H_ */
