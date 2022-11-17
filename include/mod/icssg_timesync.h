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
 * \file  icssg_timesync.h
 *
 * \brief This file contains the type definitions and helper macros for ICSSG
 *        TimeSync.
 */

/*!
 * \ingroup  ENET_MOD_TIMESYNC
 * \defgroup ICCSG_TIMESYNC_MOD ICSSG TimeSync
 *
 * The ICSSG TimeSync module implements the generic \ref ENET_MOD_TIMESYNC API
 * set.
 *
 * @{
 */

#ifndef ICSSG_TIMESYNC_H_
#define ICSSG_TIMESYNC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_mod_timesync.h>

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

/*!
 * \brief ICSSG TimeSync clock types.
 */
typedef enum IcssgTimeSync_ClkType_e
{
    /*! Working clock type */
    ICSSG_TIMESYNC_CLKTYPE_WORKING_CLOCK = 1U,

    /*! System time clock type */
    ICSSG_TIMESYNC_CLKTYPE_SYSTEM_TIME,

    /*! Global time clock type */
    ICSSG_TIMESYNC_CLKTYPE_GLOBAL_TIME,
} IcssgTimeSync_ClkType;

/*!
 * \brief TimeSync configuration parameters
 */
typedef struct IcssgTimeSync_Cfg_s
{
    /*! Whether TimeSync will be enabled or not.
     *
     *  In case of Dual-MAC, only one port can use TimeSync at a time, so it
     *  should be disabled via this config parameter.
     *  This parameter will be overwritten (to false) if it's detected that
     *  TimeSync is already is use for another Dual-MAC port.
     *
     *  There is no limitation in ICSSG Switch, it can be kept enabled. */
    bool enable;

    /*! Control variable for set/get operations */
    IcssgTimeSync_ClkType clkType;

    /*! Working clock SyncOut start time within the cycle */
    uint32_t syncOut_start_WC;

    /*! Working clock SyncOut pulse width */
    uint32_t syncOut_pwidth_WC;
} IcssgTimeSync_Cfg;

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

#endif /* ICSSG_TIMESYNC_H_ */

/*! @} */
