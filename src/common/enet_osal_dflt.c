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
 * \file  enet_osal_dflt.c
 *
 * \brief This file contains a default OSAL implementation.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include <include/common/enet_osal_dflt.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define OSAL_ENETOSAL_CONFIGNUM_HWI                                     (16)
#define OSAL_ENETOSAL_CONFIGNUM_SEMAPHORE                               (16)



/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static uintptr_t EnetOsalDflt_disableAllIntr(void);

static void EnetOsalDflt_restoreAllIntr(uintptr_t cookie);

static void *EnetOsalDflt_registerIntr(EnetOsal_Isr isrFxn,
                                       uint32_t coreIntrNum,
                                       uint32_t intrPriority,
                                       uint32_t intrTrigType,
                                       void *arg);

static void EnetOsalDflt_unregisterIntr(void *hHwi);

static void EnetOsalDflt_enableIntr(uint32_t coreIntrNum);

static void EnetOsalDflt_disableIntr(uint32_t coreIntrNum);

static void *EnetOsalDflt_createMutex(void);

static void EnetOsalDflt_deleteMutex(void *hMutex);

static void EnetOsalDflt_lockMutex(void *hMutex);

static void EnetOsalDflt_unlockMutex(void *hMutex);

static bool EnetOsalDflt_isCacheCoherent(void);

static void EnetOsalDflt_cacheInv(const void *addr,
                                  int32_t size);

static void EnetOsalDflt_cacheWb(const void *addr,
                                 int32_t size);

static void EnetOsalDflt_cacheWbInv(const void *addr,
                                    int32_t size);

static uint32_t EnetOsalDflt_timerRead();

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
uint32_t  gEnetOsalHwiAllocCnt = 0, gEnetOsalHwiPeak = 0;
uint32_t  gEnetOsalSemAllocCnt = 0, gEnetOsalSemPeak = 0;

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

/* global pool of statically allocated semaphore pools */
static HwiP_enetOsal gOsalHwiPEnetOsalPool[OSAL_ENETOSAL_CONFIGNUM_HWI];
/* global pool of statically allocated semaphore pools */
static SemaphoreP_enetOsal gOsalSemPEnetOsalPool[OSAL_ENETOSAL_CONFIGNUM_SEMAPHORE];



/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetOsalDflt_initCfg(EnetOsal_Cfg *cfg)
{
    cfg->disableAllIntr  = &EnetOsalDflt_disableAllIntr;;
    cfg->restoreAllIntr  = &EnetOsalDflt_restoreAllIntr;
    cfg->disableIntr     = &EnetOsalDflt_disableIntr;
    cfg->restoreIntr     = &EnetOsalDflt_enableIntr;
    cfg->registerIntr    = &EnetOsalDflt_registerIntr;
    cfg->unregisterIntr  = &EnetOsalDflt_unregisterIntr;
    cfg->createMutex     = &EnetOsalDflt_createMutex;
    cfg->deleteMutex     = &EnetOsalDflt_deleteMutex;
    cfg->lockMutex       = &EnetOsalDflt_lockMutex;
    cfg->unlockMutex     = &EnetOsalDflt_unlockMutex;
    cfg->isCacheCoherent = &EnetOsalDflt_isCacheCoherent;
    cfg->cacheInv        = &EnetOsalDflt_cacheInv;
    cfg->cacheWb         = &EnetOsalDflt_cacheWb;
    cfg->cacheWbInv      = &EnetOsalDflt_cacheWbInv;
    cfg->timerRead       = &EnetOsalDflt_timerRead;
}

static uintptr_t EnetOsalDflt_disableAllIntr(void)
{
    return HwiP_disable();
}

static void EnetOsalDflt_restoreAllIntr(uintptr_t cookie)
{
    HwiP_restore(cookie);
}

static void *EnetOsalDflt_registerIntr(EnetOsal_Isr isrFxn,
                                       uint32_t coreIntrNum,
                                       uint32_t intrPriority,
                                       uint32_t intrTrigType,
                                       void *arg)
{
    HwiP_Params intrPrms;
    HwiP_enetOsal    *hwiPool;
    uint32_t          maxHwi;
    uintptr_t         key;
    uint32_t          i;
    HwiP_enetOsal *handle = (HwiP_enetOsal *) NULL;
	int32_t status;

	/* Pick up the internal static memory block */
    hwiPool        = (HwiP_enetOsal *) &gOsalHwiPEnetOsalPool[0];
    maxHwi         = OSAL_ENETOSAL_CONFIGNUM_HWI;

    if(gEnetOsalHwiAllocCnt==0U) 
    {
        (void)memset((void *)gOsalHwiPEnetOsalPool,0,sizeof(gOsalHwiPEnetOsalPool));
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
        handle = (HwiP_enetOsal *) &hwiPool[i];
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
		handle = (HwiP_enetOsal *) NULL;
    }

    return handle;
}

static void EnetOsalDflt_unregisterIntr(void *hHwi)
{
    uintptr_t   key;
    HwiP_enetOsal *hwi = (HwiP_enetOsal *)hHwi;
    
    DebugP_assert((hHwi != NULL));

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

static void EnetOsalDflt_enableIntr(uint32_t coreIntrNum)
{
    HwiP_enableInt(coreIntrNum);
}

static void EnetOsalDflt_disableIntr(uint32_t coreIntrNum)
{
    HwiP_disableInt(coreIntrNum);
}

static void *EnetOsalDflt_createMutex(void)
{
    void * ret_handle;
    SemaphoreP_enetOsal *handle = (SemaphoreP_enetOsal *) NULL;
    uint32_t          i;
    uintptr_t         key;
    SemaphoreP_enetOsal *semPool;
    uint32_t          maxSemaphores;
	int32_t           status;

	/* Pick up the internal static memory block */
	semPool        = (SemaphoreP_enetOsal *) &gOsalSemPEnetOsalPool[0];
	maxSemaphores  = OSAL_ENETOSAL_CONFIGNUM_SEMAPHORE;
	
	if(gEnetOsalSemAllocCnt==0U) 
	{
		(void)memset( (void *)gOsalSemPEnetOsalPool,0,sizeof(gOsalSemPEnetOsalPool));
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
        handle = (SemaphoreP_enetOsal *) &semPool[i];
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

static void EnetOsalDflt_deleteMutex(void *handle)
{
    uintptr_t   key;
    SemaphoreP_enetOsal *semaphore = (SemaphoreP_enetOsal *)handle;

    DebugP_assert((handle != NULL));
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

static void EnetOsalDflt_lockMutex(void *hMutex)
{
    SemaphoreP_enetOsal *semaphore = (SemaphoreP_enetOsal *)hMutex;

    SemaphoreP_pend(&semaphore->sem, SystemP_WAIT_FOREVER);
}

static void EnetOsalDflt_unlockMutex(void *hMutex)
{
    SemaphoreP_enetOsal *semaphore = (SemaphoreP_enetOsal *)hMutex;

    SemaphoreP_post(&semaphore->sem);
}

static bool EnetOsalDflt_isCacheCoherent(void)
{
    bool isCoherent = false;

    return isCoherent;
}

static void EnetOsalDflt_cacheInv(const void *addr,
                                  int32_t size)
{
    CacheP_inv((void *)addr, size, CacheP_TYPE_ALLD);
}

static void EnetOsalDflt_cacheWb(const void *addr,
                                 int32_t size)
{
    CacheP_wb((void *)addr, size, CacheP_TYPE_ALLD);
}

static void EnetOsalDflt_cacheWbInv(const void *addr,
                                    int32_t size)
{
    CacheP_wbInv((void *)addr, size, CacheP_TYPE_ALLD);
}

static uint32_t EnetOsalDflt_timerRead()
{
    return 0;
}
