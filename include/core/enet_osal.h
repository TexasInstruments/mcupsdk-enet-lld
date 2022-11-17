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
 * \brief Enet OSAL intr disable function prototype.
 *
 * \return Cookie to be passed back to enable interrupt function
 */
typedef uintptr_t (*EnetOsal_DisableAllIntr)(void);

/*!
 * \brief Enet OSAL intr restore function prototype.
 *
 * \param cookie  This is returned in disable interrupt function
 */
typedef void (*EnetOsal_RestoreAllIntr)(uintptr_t cookie);

/*!
 * \brief Enet OSAL intr restore function prototype.
 *
 * \param coreIntrNum  Interrupt to restore
 */
typedef void (*EnetOsal_RestoreIntr)(uint32_t coreIntrNum);

/*!
 * \brief Enet OSAL intr disable function prototype.
 *
 * \param coreIntrNum  Interrupt to disable
 */
typedef void (*EnetOsal_DisableIntr)(uint32_t coreIntrNum);

/*!
 * \brief Enet OSAL ISR callback function prototype.
 *
 * \param arg     App data
 */
typedef void (*EnetOsal_Isr)(uintptr_t arg);

/*!
 * \brief Enet OSAL ISR register function prototype.
 *
 * \param isrFxn       ISR callback fxn pointer
 * \param coreIntrNum  Core interrupt number to register
 * \param intrPriority Priority
 * \param triggerType  interrupt trigger type for ARM corepac as
 *                     \ref EnetOSAL_armGicTrigType_t
 * \param arg          Arg that will be passed back in the ISR
 *
 * \return Created Hwi handle
 */
typedef void *(*EnetOsal_RegisterIntr)(EnetOsal_Isr isrFxn,
                                       uint32_t coreIntrNum,
                                       uint32_t intrPriority,
                                       uint32_t triggerType,
                                       void *arg);

/*!
 * \brief Enet OSAL ISR unregister function prototype.
 *
 * \param hHwi    Hwi handle
 */
typedef void (*EnetOsal_UnregisterIntr)(void *hHwi);

/*!
 * \brief Enet OSAL mutex create function prototype to protect
 *        critical section.
 *
 * \return Pointer to mutex object
 */
typedef void *(*EnetOsal_CreateMutex)(void);

/*!
 * \brief Enet OSAL mutex delete function prototype.
 *
 * \param hMutex  Pointer to mutex object returned during create
 */
typedef void (*EnetOsal_DeleteMutex)(void *hMutex);

/*!
 * \brief Enet OSAL mutex lock function prototype.
 *
 * \param hMutex  Pointer to mutex object returned during create
 */
typedef void (*EnetOsal_LockMutex)(void *hMutex);

/*!
 * \brief Enet OSAL mutex lock function prototype.
 *
 * \param hMutex  Pointer to mutex object returned during create
 */
typedef void (*EnetOsal_UnlockMutex)(void *hMutex);

/*!
 * \brief Enet OSAL cache coherency check function prototype.
 *
 * \return Whether cache is coherent or not
 */
typedef bool (*EnetOsal_IsCacheCoherent)(void);

/*!
 * \brief Enet OSAL cache invalidate function prototype.
 *
 * \param addr    Start address of the cache line/s
 * \param size    Size (in bytes) of the memory to invalidate
 */
typedef void (*EnetOsal_CacheInv)(const void *addr,
                                  int32_t size);

/*!
 * \brief Enet OSAL cache writeback function prototype.
 *
 * \param addr    Start address of the cache line/s
 * \param size    Size (in bytes) of the memory to be written back
 */
typedef void (*EnetOsal_CacheWb)(const void *addr,
                                 int32_t size);

/*!
 * \brief Enet OSAL cache writeback invalidate function prototype.
 *
 * \param addr   Start address of the cache line/s
 * \param size   Size (in bytes) of the memory to be written back
 */
typedef void (*EnetOsal_CacheWbInv)(const void *addr,
                                    int32_t size);

/*!
 * \brief Enet OSAL timer read function prototype
 *
 * \return Timer's read value
 */
typedef uint32_t (*EnetOsal_TimerRead)(void);

/*!
 * \brief Enet OSAL configuration paramters.
 */
typedef struct EnetOsal_Cfg_s
{
    /*! All interrupt disable function pointer */
    EnetOsal_DisableAllIntr disableAllIntr;

    /*! All interrupt restore function pointer */
    EnetOsal_RestoreAllIntr restoreAllIntr;

    /*! Interrupt disable function pointer */
    EnetOsal_DisableIntr disableIntr;

    /*! Interrupt restore function pointer */
    EnetOsal_RestoreIntr restoreIntr;

    /*! Register interrupt function pointer */
    EnetOsal_RegisterIntr registerIntr;

    /*! Unregister interrupt function pointer */
    EnetOsal_UnregisterIntr unregisterIntr;

    /*! Create mutex function pointer */
    EnetOsal_CreateMutex createMutex;

    /*! Delete mutex function pointer */
    EnetOsal_DeleteMutex deleteMutex;

    /*! Lock mutex function pointer */
    EnetOsal_LockMutex lockMutex;

    /*! Unlock mutex function pointer */
    EnetOsal_UnlockMutex unlockMutex;

    /*! Cache coherency function pointer */
    EnetOsal_IsCacheCoherent isCacheCoherent;

    /*! Cache invalidate function pointer */
    EnetOsal_CacheInv cacheInv;

    /*! Cache write-back function pointer */
    EnetOsal_CacheWb cacheWb;

    /*! Cache invalidate and write-back function pointer */
    EnetOsal_CacheWbInv cacheWbInv;

    /*! Timer read function pointer */
    EnetOsal_TimerRead timerRead;
} EnetOsal_Cfg;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize OSAL with provided configuration.
 *
 * Initializes the Enet driver OSAL with the provided configuration.  It's
 * expected that all function pointers in the configuration are not NULL.
 *
 * \param cfg   Configuration parameters
 */
void EnetOsal_init(const EnetOsal_Cfg *cfg);

/*!
 * \brief Deinitialize OSAL.
 *
 * Deinitializes the Enet driver OSAL by clearing the configuration previously
 * set when calling EnetOsal_init().
 */
void EnetOsal_deinit(void);

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
void *EnetOsal_registerIntr(EnetOsal_Isr isrFxn,
                            uint32_t coreIntrNum,
                            uint32_t intrPriority,
                            uint32_t intrTrigType,
                            void *arg);

/*!
 * \brief Unregister an interrupt.
 *
 * \param hHwi    Interrupt handle pointer
 */
void EnetOsal_unregisterIntr(void *hHwi);

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
void *EnetOsal_createMutex(void);

/*!
 * \brief Delete a mutex.
 *
 * \param hMutex   Mutex handle pointer
 */
void EnetOsal_deleteMutex(void *hMutex);

/*!
 * \brief Lock a mutex.
 *
 * \param hMutex   Mutex handle pointer
 */
void EnetOsal_lockMutex(void *hMutex);

/*!
 * \brief Unlock a mutex.
 *
 * \param hMutex   Mutex handle pointer
 */
void EnetOsal_unlockMutex(void *hMutex);

/*!
 * \brief Invalidate cache.
 *
 * Invalidates cache in a range of memory.
 *
 * \param addr    Start address of the cache line(s)
 * \param size    Size (in bytes) of the memory to invalidate
 */
void EnetOsal_cacheInv(const void *addr,
                           int32_t size);

/*!
 * \brief Write-back cache.
 *
 * Writes back cache a range of memory from cache.
 *
 * \param addr    Start address of the cache line(s)
 * \param size    Size (in bytes) of the memory to be written back
 */
void EnetOsal_cacheWb(const void *addr,
                          int32_t size);

/*!
 * \brief Write-back and invalidate cache.
 *
 * Writes back and invalidates a range of memory.
 *
 * \param addr   Start address of the cache line/s
 * \param size   Size (in bytes) of the memory to be written back
 */
void EnetOsal_cacheWbInv(const void *addr,
                             int32_t size);

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
