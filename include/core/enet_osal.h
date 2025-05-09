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
 * \file  enet_osal.h
 *
 * \brief This file contains the OSAL API of the Enet driver.
 */

#ifndef ENET_OSAL_H_
#define ENET_OSAL_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include <kernel/dpl/HwiP.h>
 #include <kernel/dpl/SystemP.h>
 #include <kernel/dpl/SemaphoreP.h>
 #include <kernel/dpl/CacheP.h>

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
 *  @brief    Semaphore structure
 */
typedef struct SemaphoreP_enetOsal_s {
    bool              used;
    SemaphoreP_Object  sem;
} SemaphoreP_enetOsal;

/*!
 *  @brief    Hwi structure
 */
typedef struct HwiP_enetOsal_s {
    bool              used;
    HwiP_Object       hwi;
} HwiP_enetOsal;

/**
 *  @brief Enumerates the types different trigger types.
 *  Please refer to Section 4.3.13 Interrupt Configuration Registers, GICD_ICFGRn
 *  of ARM Generic Interrupt Controller Architecture version 2.0
 *  Architecture Specification document for details.
 */
typedef enum
{
    ENETOSAL_ARM_GIC_TRIG_TYPE_LEVEL = 1,
    /**< Corresponding interrupt is level sensitive */

    ENETOSAL_ARM_GIC_TRIG_TYPE_EDGE = 2,
    /**< Corresponding interrupt is edge sensitive */

    ENETOSAL_ARM_GIC_TRIG_TYPE_HIGH_LEVEL = 3,
    /**< Coressponding interrupt is high level sensitive */

    ENETOSAL_ARM_GIC_TRIG_TYPE_LOW_LEVEL = 4,
    /**< Coressponding interrupt is low level sensitive */

    ENETOSAL_ARM_GIC_TRIG_TYPE_RISING_EDGE = 5,
    /**< Coressponding interrupt is rising edge sensitive */

    ENETOSAL_ARM_GIC_TRIG_TYPE_FALLING_EDGE = 6
    /**< Coressponding interrupt is falling edge sensitive */
} EnetOSAL_armGicTrigType_t;

/*!
 * \brief Enet OSAL ISR callback function prototype.
 *
 * \param arg     App data
 */
typedef void (*EnetOsal_Isr)(uintptr_t arg);

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Enet OSAL intr disable function prototype.
 *
 * \return Cookie to be passed back to enable interrupt function
 */
uintptr_t EnetOsal_disableAllIntr(void);

/*!
 * \brief Enet OSAL intr restore function prototype.
 *
 * \param cookie  This is returned in disable interrupt function
 */
void EnetOsal_restoreAllIntr(uintptr_t cookie);

/*!
 * \brief Register an ISR for an interrupt.
 *
 * \param isrFxn        Interrupt service routine
 * \param coreIntrNum   Interrupt number
 * \param intrPriority  Interrupt priority
 * \param intrTrigType  Interrupt trigger type
 * \param arg           Argument to ISR function
 *
 * \return Interrupt handle pointer
 */
HwiP_enetOsal * EnetOsal_registerIntr(EnetOsal_Isr isrFxn,
                            uint32_t coreIntrNum,
                            uint32_t intrPriority,
                            uint32_t intrTrigType,
                            void *arg);

/*!
 * \brief Unregister an interrupt.
 *
 * \param hHwi    Interrupt handle pointer
 */
void EnetOsal_unregisterIntr(HwiP_enetOsal * hHwi);

/*!
 * \brief Enable interrupt.
 *
 * \param coreIntrNum   Interrupt number
 */
void EnetOsal_enableIntr(uint32_t coreIntrNum);

/*!
 * \brief Disable interrupt.
 *
 * \param coreIntrNum   Interrupt number
 */
void EnetOsal_disableIntr(uint32_t coreIntrNum);

/*!
 * \brief Create a mutex.
 *
 * \return Mutex handle pointer
 */
SemaphoreP_enetOsal * EnetOsal_createMutex(void);

/*!
 * \brief Delete a mutex.
 *
 * \param hMutex   Mutex handle pointer
 */
void EnetOsal_deleteMutex(SemaphoreP_enetOsal * hMutex);

/*!
 * \brief Lock a mutex.
 *
 * \param hMutex   Mutex handle pointer
 */
void EnetOsal_lockMutex(SemaphoreP_enetOsal * hMutex);

/*!
 * \brief Unlock a mutex.
 *
 * \param hMutex   Mutex handle pointer
 */
void EnetOsal_unlockMutex(SemaphoreP_enetOsal * hMutex);

/*!
 * \brief Invalidate cache.
 *
 * Invalidates cache in a range of memory.
 *
 * \param addr    Start address of the cache line(s)
 * \param size    Size (in bytes) of the memory to invalidate
 */
void EnetOsal_cacheInv(void *addr,
                           int32_t size);

/*!
 * \brief Write-back cache.
 *
 * Writes back cache a range of memory from cache.
 *
 * \param addr    Start address of the cache line(s)
 * \param size    Size (in bytes) of the memory to be written back
 */
void EnetOsal_cacheWb(void *addr,
                          int32_t size);

/*!
 * \brief Write-back and invalidate cache.
 *
 * Writes back and invalidates a range of memory.
 *
 * \param addr   Start address of the cache line/s
 * \param size   Size (in bytes) of the memory to be written back
 */
void EnetOsal_cacheWbInv(void *addr,
                             int32_t size);

bool EnetOsal_isCacheCoherent(void);

/*!
 * \brief Get the time difference with respect to timestamp.
 *
 * Gets the difference between time value passed to this function and current
 * time from timer.
 *
 * \return Time difference
 */
uint32_t EnetOsal_timerGetDiff(uint32_t startTime);

/*!
 * \brief Read current timer value.
 *
 * \return Timer's read value
 */
uint32_t EnetOsal_timerRead(void);

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* ENET_OSAL_H_ */
