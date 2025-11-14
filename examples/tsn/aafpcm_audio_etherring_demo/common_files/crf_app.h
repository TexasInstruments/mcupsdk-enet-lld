/*
 *  Copyright (c) Texas Instruments Incorporated 2025
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

#ifndef __CRF_APP_H__
#define __CRF_APP_H__

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include "avtpc_crf.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    uint8_t streamID[8];
    uint32_t baseFrequency;
    uint16_t timestampingInterval;
    uint16_t vlanID;
    uint8_t vlanPCP;
    bool isListener;
}crfApp_crfConfig;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 * @brief Initializes the CRF application task.
 *
 * This function initializes CRF Task and
 * Calculates initial Media Clock measurement offset
 */
void crfApp_initTask(void);

/**
 * @brief Runs the CRF application task.
 *
 * This function executes the main logic of the CRF application task.
 *
 * @param args Pointer to arguments required by the task.
 */
void crfApp_runTask(void* args);

/**
 * @brief Initializes the CRF application with the specified configuration.
 *
 * @param config Pointer to the \ref crfApp_crfConfig
 * @return Returns 0 on success, or a negative error code on failure.
 */
int32_t crfApp_init(crfApp_crfConfig *config);

/**
 * @brief Callback function for receiving AVBTP payload data.
 *
 * This is a Hook function that has to be called to provide timestamps to Crf App.
 *
 * @param payload Pointer to the received payload data.
 * @param payload_size Size of the received payload in bytes.
 * @param cbinfo Pointer to the AVBTP receive callback information structure.
 * @param cbdata User-defined callback data.
 * @return Returns 0 on success, or a negative error code on failure.
 */
int crfApp_rxCallback(uint8_t *payload, int payload_size,
                      avbtp_rcv_cb_info_t *cbinfo, void *cbdata);

/**
 * @brief Handles periodic tick events based on the media clock edge.
 *
 * This function should be called at each media clock edge to perform
 * time-based processing in the CRF application.
 *
 * @param mediaClockEdge The current media clock edge timestamp.
 */
void crfApp_tick(uint64_t mediaClockEdge);

/**
 * @brief Retrieves the frequency stability status.
 *
 * This function is used to get the frequency stable status
 *
 * @return true if the frequency is stable, false otherwise.
 */
bool crfApp_getFreqStableStatus(void);

#endif /* __CRF_APP_H__ */
