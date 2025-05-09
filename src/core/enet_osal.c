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

#define OSAL_ENETOSAL_CONFIGNUM_HWI                                     (16)
#define OSAL_ENETOSAL_CONFIGNUM_SEMAPHORE                               (16)



/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */



/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

uint32_t  gEnetOsalHwiAllocCnt = 0, gEnetOsalHwiPeak = 0;
uint32_t  gEnetOsalSemAllocCnt = 0, gEnetOsalSemPeak = 0;

/* global pool of statically allocated semaphore pools */
static HwiP_enetOsal gOsalHwiPEnetOsalPool[OSAL_ENETOSAL_CONFIGNUM_HWI];
/* global pool of statically allocated semaphore pools */
static SemaphoreP_enetOsal gOsalSemPEnetOsalPool[OSAL_ENETOSAL_CONFIGNUM_SEMAPHORE];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
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
    return HwiP_disable();
}

void EnetOsal_restoreAllIntr(uintptr_t cookie)
{
    HwiP_restore(cookie);
}

#endif

HwiP_enetOsal * EnetOsal_registerIntr(EnetOsal_Isr isrFxn,
                                       uint32_t coreIntrNum,
                                       uint32_t intrPriority,
                                       uint32_t intrTrigType,
                                       void *arg)
{
    HwiP_Params intrPrms;
    HwiP_enetOsal    *hwiPool = NULL;
    uint32_t          maxHwi;
    uintptr_t         key;
    uint32_t          i;
    HwiP_enetOsal *handle = NULL;
	int32_t status;

	/* Pick up the internal static memory block */
    hwiPool        = &gOsalHwiPEnetOsalPool[0];
    maxHwi         = OSAL_ENETOSAL_CONFIGNUM_HWI;

    if(gEnetOsalHwiAllocCnt==0U) 
    {
        memset(gOsalHwiPEnetOsalPool,0,sizeof(gOsalHwiPEnetOsalPool));
    }
    /* Grab the memory */
    key = HwiP_disable();

    for (i = 0U; i < maxHwi; i++)
    {
        if (hwiPool[i].used == false)
        {
            hwiPool[i].used = true;
            /* Update statistics */
            gEnetOsalHwiAllocCnt++;
            if (gEnetOsalHwiAllocCnt > gEnetOsalHwiPeak)
            {
                gEnetOsalHwiPeak = gEnetOsalHwiAllocCnt;
            }
            break;
        }
    }
    HwiP_restore(key);

    if (i < maxHwi)
    {
        /* Grab the memory */
        handle = &hwiPool[i];
    }

    HwiP_Params_init(&intrPrms);

    /* Populate the interrupt parameters */
    intrPrms.args          = arg;
    intrPrms.callback      = (HwiP_FxnCallback)isrFxn;
    intrPrms.priority        = intrPriority;
    if (intrTrigType == ENETOSAL_ARM_GIC_TRIG_TYPE_EDGE)
    {
        intrPrms.isPulse = 1;
    }
    else
    {
        intrPrms.isPulse = 0;
    }
    intrPrms.eventId       = coreIntrNum; //only used with c6x with event combiner
	intrPrms.isFIQ         = false;
    intrPrms.intNum        = coreIntrNum;

    /* Register interrupts */
    status = HwiP_construct(&handle->hwi, &intrPrms);
    if (status != SystemP_SUCCESS)
    {
		/* Free the allocated memory and return null */
		handle->used = false;
		handle = NULL;
    }

    return handle;
}

void EnetOsal_unregisterIntr(HwiP_enetOsal * hwi)
{
    uintptr_t   key;
    
    DebugP_assert((hwi != NULL));

    if ((hwi!=NULL) && (hwi->used==true)) {
      HwiP_destruct(&hwi->hwi);
      key = HwiP_disable();
      hwi->used = false;
      /* Found the osal hwi object to delete */
      if (gEnetOsalHwiAllocCnt > 0U)
      {
        gEnetOsalHwiAllocCnt--;
      }

      HwiP_restore(key);
    }
}

void EnetOsal_enableIntr(uint32_t coreIntrNum)
{
    HwiP_enableInt(coreIntrNum);
}

void EnetOsal_disableIntr(uint32_t coreIntrNum)
{
    HwiP_disableInt(coreIntrNum);
}
SemaphoreP_enetOsal * EnetOsal_createMutex(void)
{
    SemaphoreP_enetOsal * ret_handle = NULL;
    SemaphoreP_enetOsal *handle = NULL;
    uint32_t          i;
    uintptr_t         key;
    SemaphoreP_enetOsal *semPool = NULL;
    uint32_t          maxSemaphores;
	int32_t           status;

	/* Pick up the internal static memory block */
	semPool        = &gOsalSemPEnetOsalPool[0];
	maxSemaphores  = OSAL_ENETOSAL_CONFIGNUM_SEMAPHORE;
	
	if(gEnetOsalSemAllocCnt==0U) 
	{
		memset(gOsalSemPEnetOsalPool,0,sizeof(gOsalSemPEnetOsalPool));
	}

    key = HwiP_disable();

     for (i = 0; i < maxSemaphores; i++)
     {
         if (semPool[i].used == false)
         {
             semPool[i].used = true;
             /* Update statistics */
             gEnetOsalSemAllocCnt++;
             if (gEnetOsalSemAllocCnt > gEnetOsalSemPeak)
             {
                 gEnetOsalSemPeak = gEnetOsalSemAllocCnt;
             }
             break;
         }
     }
     HwiP_restore(key);

    if (i < maxSemaphores)
    {
        /* Grab the memory */
        handle = &semPool[i];
    }

    if (handle == NULL) {
        ret_handle = NULL;
    }
    else
    {
        status = SemaphoreP_constructMutex(&handle->sem);
		if (status == SystemP_SUCCESS)
		{
			ret_handle = handle;
		}
		else
		{
			ret_handle = NULL;
		}
    }
    return ret_handle;
}

void EnetOsal_deleteMutex(SemaphoreP_enetOsal * semaphore)
{
    uintptr_t   key;

    DebugP_assert((semaphore != NULL));
    if((semaphore != NULL) && (semaphore->used==true))
    {
        SemaphoreP_destruct(&semaphore->sem);

        key = HwiP_disable();
        semaphore->used = false;
        /* Found the osal semaphore object to delete */
        if (gEnetOsalSemAllocCnt > 0U)
        {
            gEnetOsalSemAllocCnt--;
        }
        HwiP_restore(key);
    } 
}
void EnetOsal_lockMutex(SemaphoreP_enetOsal * semaphore)
{
    SemaphoreP_pend(&semaphore->sem, SystemP_WAIT_FOREVER);
}

void EnetOsal_unlockMutex(SemaphoreP_enetOsal * semaphore)
{
    SemaphoreP_post(&semaphore->sem);
}

bool EnetOsal_isCacheCoherent(void)
{
    bool isCoherent = false;

    return isCoherent;
}

void EnetOsal_cacheInv(void *addr,
                                  int32_t size)
{
    CacheP_inv(addr, size, CacheP_TYPE_ALLD);
}

void EnetOsal_cacheWb(void *addr,
                                 int32_t size)
{
    CacheP_wb(addr, size, CacheP_TYPE_ALLD);
}

void EnetOsal_cacheWbInv(void *addr,
                                    int32_t size)
{
    CacheP_wbInv(addr, size, CacheP_TYPE_ALLD);
}

uint32_t EnetOsal_timerRead()
{
    return 0;
}

uint32_t EnetOsal_timerGetDiff(uint32_t startTime)
{
    uint32_t currTime;

    currTime = EnetOsal_timerRead();

    return(currTime - startTime);
}


