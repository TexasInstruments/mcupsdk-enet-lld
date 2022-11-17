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
 * \file  enet_base.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Enet Peripheral interface.
 */

#ifndef ENET_BASE_H_
#define ENET_BASE_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <include/core/enet_types.h>
#include <include/core/enet_per.h>
#include <include/core/enet_trace.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Feature mask which indicates that all of the configurable features
 *         are enabled. */
#define ENET_FEAT_ALL                         (0xFFFFFFFFU)

/*! \brief Feature masks which indicates that none of the configurable features
 *         are enabled. */
#define ENET_FEAT_BASE                        (0x00000000U)

/*! \brief Check if a configurable feature is enabled or not. */
#define ENET_FEAT_IS_EN(feats, mask)          (((feats) & (mask)) != 0U)

/*! \brief Errata mask which indicates that none of the erratas is applicable. */
#define ENET_ERRATA_NONE                      (0x00000000U)

/*! \brief Check if any errata is applicable, from an errata mask. */
#define ENET_ERRATA_IS_ANY(mask)              ((mask) != ENET_ERRATA_NONE)

/*! \brief Check if an errata is applicable. */
#define ENET_ERRATA_IS_EN(erratas, mask)      (((erratas) & (mask)) != 0U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief Enet object.
 */
typedef struct Enet_Obj_s
{
    /*! Underlying Ethernet Peripheral */
    EnetPer_Obj *enetPer;

    /*! Magic number used to indicate if driver has been opened */
    Enet_Magic magic;

    /*! Main, API-level Lock */
    void *lock;
} Enet_Obj;

/*!
 * \brief Ethernet driver handle.
 *
 * Ethernet driver opaque handle used to call any Enet related APIs.
 */
typedef struct Enet_Obj_s *Enet_Handle;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Get the Enet Peripheral handle associated with Enet driver.
 *
 * Gets the underlying Enet Peripheral handle associated with the given Enet
 * driver.
 *
 * \param hEnet      Enet handle
 *
 * \return EthPer handle
 */
static inline EnetPer_Handle Enet_getPerHandle(Enet_Handle hEnet);

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline EnetPer_Handle Enet_getPerHandle(Enet_Handle hEnet)
{
    return hEnet->enetPer;
}

#ifdef __cplusplus
}
#endif

#endif /* ENET_BASE_H_ */
