/*
 *  Copyright (C) Texas Instruments Incorporated 2026
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
 * \file clocksync.h
 *
 * \brief Synchronizes a Timer to CPTS so that the fetching of gPTP clock can
 *        be done quickly. Singleton, no RTOS dependency - only the
 *        application's #ClockSync_HwPushCallbackHook implementation may use
 *        RTOS primitives.
 */

#ifndef CLOCKSYNC_H_
#define CLOCKSYNC_H_

#include <stdint.h>
#include <enet.h>
#include <include/mod/cpsw_cpts.h>


#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \brief Initialize ClockSync.
 *
 * Connects the given timer's PWM output to the given CPTS HW push event via
 * the TimeSyncRouter, configures and starts the timer, and registers
 * ClockSync's internal callback for the CPTS HW push event.
 *
 * \param hEnet          Enet driver handle, used for CPTS IOCTL calls
 * \param coreId         Caller's core id, used for CPTS IOCTL calls
 * \param timerInstance  Logical timer instance to synchronize to CPTS
 * \param pushNum        CPTS HW push instance to synchronize on
 *
 * \return #ENET_EFAIL for error, #ENET_SOK otherwise
 */
int32_t ClockSync_init(Enet_Handle hEnet, uint32_t coreId, int32_t timerInstance, CpswCpts_HwPush pushNum);

/*!
 * \brief Get the current CPTS-synced time.
 *
 * \return #ENET_EFAIL for error, time in nanoseconds otherwise
 */
int64_t ClockSync_getCurrentTimeNs(void);

/*!
 * \brief Get the sync status.
 *
 * \return #ENET_EFAIL for error, 0 for not synced, 1 for synced
 */
int32_t ClockSync_getSyncStatus(void);

/*!
 * \brief Run the sync state machine one step: reads the latest CPTS HW push
 *        event timestamp and the current timer count, and updates m, c and
 *        Error.
 *
 * \return #ENET_EFAIL for error, #ENET_SOK otherwise
 */
int32_t ClockSync_tick(void);

/*!
 * \brief Weak hook invoked by ClockSync's internal CPTS HW push callback.
 *
 * The application must define this function. ClockSync itself has no RTOS
 * dependency - it is only this application-provided hook that may use RTOS
 * primitives (e.g. post a semaphore to a task, or call #ClockSync_tick
 * directly from ISR context).
 */
void ClockSync_HwPushCallbackHook(void);

/*!
 * \brief Stop the timer and disconnect the TimeSyncRouter connection.
 */
void ClockSync_deInit(void);

/*!
 * \brief Force the sync flag false.
 *
 * Called by the application when it detects no HW push callback for some
 * period of time. ClockSync itself does not detect this timeout - the
 * caller is responsible for that. Cleared back to synced again by the next
 * #ClockSync_tick that lands within the sync threshold.
 */
void ClockSync_timeout(void);

#ifdef __cplusplus
}
#endif

#endif /* CLOCKSYNC_H_ */
