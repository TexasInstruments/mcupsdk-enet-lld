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
 * \file  enet_osal.c
 *
 * \brief This file contains the OSAL wrapper implementation.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <include/core/enet_types.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_osal.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/*!
 * \brief Enet OSAL object.
 */
typedef struct EnetOsal_Obj_s
{
    /*! Disable all interrupts function pointer */
    EnetOsal_DisableAllIntr disableAllIntr;

    /*! Restore all interrupts function pointer */
    EnetOsal_RestoreAllIntr restoreAllIntr;

    /*! Disable interrupt function pointer */
    EnetOsal_DisableIntr disableIntr;

    /*! Restore interrupt function pointer */
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

    /*! Cache coherency check function pointer */
    EnetOsal_IsCacheCoherent isCacheCoherent;

    /*! Cache invalidate function pointer */
    EnetOsal_CacheInv cacheInv;

    /*! Cache write-back function pointer */
    EnetOsal_CacheWb cacheWb;

    /*! Cache invalidate and write-back function pointer */
    EnetOsal_CacheWbInv cacheWbInv;

    /*! Timer read function pointer */
    EnetOsal_TimerRead timerRead;
} EnetOsal_Obj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

EnetOsal_Obj gEnetOsalObj;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetOsal_init(const EnetOsal_Cfg *cfg)
{
    memset(&gEnetOsalObj, 0, sizeof(gEnetOsalObj));

    Enet_devAssert(cfg->disableAllIntr != NULL, "Invalid disableAllIntr pointer\n");
    Enet_devAssert(cfg->restoreAllIntr != NULL, "Invalid restoreAllIntr pointer\n");
    Enet_devAssert(cfg->disableIntr != NULL, "Invalid disableIntr pointer\n");
    Enet_devAssert(cfg->restoreIntr != NULL, "Invalid restoreIntr pointer\n");
    Enet_devAssert(cfg->registerIntr != NULL, "Invalid registerIntr pointer\n");
    Enet_devAssert(cfg->unregisterIntr != NULL, "Invalid unregisterIntr pointer\n");
    Enet_devAssert(cfg->createMutex != NULL, "Invalid createMutex pointer\n");
    Enet_devAssert(cfg->deleteMutex != NULL, "Invalid deleteMutex pointer\n");
    Enet_devAssert(cfg->lockMutex != NULL, "Invalid lockMutex pointer\n");
    Enet_devAssert(cfg->unlockMutex != NULL, "Invalid unlockMutex pointer\n");
    Enet_devAssert(cfg->cacheInv != NULL, "Invalid cacheInv pointer\n");
    Enet_devAssert(cfg->cacheWb != NULL, "Invalid cacheWb pointer\n");
    Enet_devAssert(cfg->cacheWbInv != NULL, "Invalid cacheWbInv pointer\n");
    Enet_devAssert(cfg->isCacheCoherent != NULL, "Invalid isCacheCoherent pointer\n");
    Enet_devAssert(cfg->cacheInv != NULL, "Invalid cacheInv pointer\n");
    Enet_devAssert(cfg->cacheWb != NULL, "Invalid cacheWb pointer\n");
    Enet_devAssert(cfg->timerRead != NULL, "Invalid timerRead pointer\n");

    gEnetOsalObj.disableAllIntr  = cfg->disableAllIntr;
    gEnetOsalObj.restoreAllIntr  = cfg->restoreAllIntr;
    gEnetOsalObj.disableIntr     = cfg->disableIntr;
    gEnetOsalObj.restoreIntr     = cfg->restoreIntr;
    gEnetOsalObj.registerIntr    = cfg->registerIntr;
    gEnetOsalObj.unregisterIntr  = cfg->unregisterIntr;
    gEnetOsalObj.createMutex     = cfg->createMutex;
    gEnetOsalObj.deleteMutex     = cfg->deleteMutex;
    gEnetOsalObj.lockMutex       = cfg->lockMutex;
    gEnetOsalObj.unlockMutex     = cfg->unlockMutex;
    gEnetOsalObj.isCacheCoherent = cfg->isCacheCoherent;
    gEnetOsalObj.cacheInv        = cfg->cacheInv;
    gEnetOsalObj.cacheWb         = cfg->cacheWb;
    gEnetOsalObj.cacheWbInv      = cfg->cacheWbInv;
    gEnetOsalObj.timerRead       = cfg->timerRead;
}

void EnetOsal_deinit(void)
{
    memset(&gEnetOsalObj, 0, sizeof(gEnetOsalObj));
}

#if (((__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R')) && (ENET_CFG_USE_OPTIMIZED_IRQ_CRITICAL_SECTION == 1))
/*
 *  ======== Hwi_disable ========
 */
static inline uintptr_t HwiR5_disable()
{
    uintptr_t key;
    /*
     * Note: Inline assembly follows GNU format and may not
     *       be compatible with TI compiler.
     */
    __asm__ __volatile__ (
            "mrs %0, apsr\n\t"
            "cpsid i"
            : "=r" (key)
            :: "cc", "memory"
            );
    return (key);
}

/*
 *  ======== Hwi_enable ========
 */
static inline uintptr_t HwiR5_enable()
{
    uintptr_t key;
    /*
     * Note: Inline assembly follows GNU format and may not
     *       be compatible with TI compiler.
     */
    __asm__ __volatile__ (
            "mrs %0, apsr\n\t"
            "cpsie i"
            : "=r" (key)
            :: "cc", "memory"
            );
    return (key);
}

/*
 *  ======== Hwi_restore ========
 */
static inline void HwiR5_restore(uintptr_t key)
{
    __asm__ __volatile__ (
            "msr cpsr_fc, %0"
            :: "r" (key)
            : "cc", "memory"
            );
}


uintptr_t EnetOsal_disableAllIntr(void)
{
    return HwiR5_disable();
}

void EnetOsal_restoreAllIntr(uintptr_t cookie)
{
    HwiR5_restore(cookie);
}

#else

uintptr_t EnetOsal_disableAllIntr(void)
{
    return gEnetOsalObj.disableAllIntr();
}

void EnetOsal_restoreAllIntr(uintptr_t cookie)
{
    gEnetOsalObj.restoreAllIntr(cookie);
}
#endif


void *EnetOsal_registerIntr(EnetOsal_Isr isrFxn,
                            uint32_t coreIntrNum,
                            uint32_t intrPriority,
                            uint32_t intrTrigType,
                            void *arg)
{
    return gEnetOsalObj.registerIntr(isrFxn,
                                     coreIntrNum,
                                     intrPriority,
                                     intrTrigType,
                                     arg);
}

void EnetOsal_unregisterIntr(void *hHwi)
{
    return gEnetOsalObj.unregisterIntr(hHwi);
}

void EnetOsal_restoreIntr(uint32_t coreIntrNum)
{
    gEnetOsalObj.restoreIntr(coreIntrNum);
}

void EnetOsal_disableIntr(uint32_t coreIntrNum)
{
    gEnetOsalObj.disableIntr(coreIntrNum);
}

void *EnetOsal_createMutex(void)
{
    return gEnetOsalObj.createMutex();
}

void EnetOsal_deleteMutex(void *hMutex)
{
    gEnetOsalObj.deleteMutex(hMutex);
}

void EnetOsal_lockMutex(void *hMutex)
{
    gEnetOsalObj.lockMutex(hMutex);
}

void EnetOsal_unlockMutex(void *hMutex)
{
    gEnetOsalObj.unlockMutex(hMutex);
}

bool EnetOsal_isCacheCoherent(void)
{
    return gEnetOsalObj.isCacheCoherent();
}

void EnetOsal_cacheInv(const void *pVirtAddr,
                       int32_t size)
{
    bool isCacheCoherent = gEnetOsalObj.isCacheCoherent();

    if (isCacheCoherent != true)
    {
        gEnetOsalObj.cacheInv(pVirtAddr, size);
    }
}

void EnetOsal_cacheWb(const void *pVirtAddr,
                      int32_t size)
{
    bool isCacheCoherent = gEnetOsalObj.isCacheCoherent();

    if (isCacheCoherent != true)
    {
        gEnetOsalObj.cacheWb(pVirtAddr, size);
    }
}

void EnetOsal_cacheWbInv(const void *pVirtAddr,
                         int32_t size)
{
    bool isCacheCoherent = gEnetOsalObj.isCacheCoherent();

    if (isCacheCoherent != true)
    {
        gEnetOsalObj.cacheWbInv(pVirtAddr, size);
    }
}

uint32_t EnetOsal_timerGetDiff(uint32_t startTime)
{
    uint32_t currTime;

    currTime = gEnetOsalObj.timerRead();

    return(currTime - startTime);
}

uint32_t EnetOsal_timerRead(void)
{
    return gEnetOsalObj.timerRead();
}
